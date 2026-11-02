"""v0.3有限Hertz带包络兼容工具；不作为当前LTCA通过程压力显示。

新写实现。一个基节周期内各相邻齿对按材料展长拼接，等价于相同齿、
周期稳态下一个齿的完整通过程。瞬时图只保留选定相位的一对齿。
局部有限带采用原条带Hertz半椭圆，不新增二维弹性接触平衡解。
该包络与圆柱LTCA的材料坐标峰压插值面不是同一物理量；正式显示使用
robot_joint.contact_display，瞬时图在局部沿线/垂线坐标单独呈现。
"""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import math
import numpy as np
from gear_contact.io import write_json


@dataclass
class ContactPattern:
    """SI压力矩阵为[齿廓采样, 齿宽条带]；瞬时与扫掠各有独立齿廓坐标。"""
    arrays: dict
    summary: dict


def _integer(value, name, minimum):
    if isinstance(value, bool) or not isinstance(value, (int, np.integer)) or value < minimum:
        raise ValueError(name+'必须为不小于'+str(minimum)+'的整数')


def interior_snapshot(result):
    """选择两轮有限带均位于有效齿面内部的最重载齿对，避开未建模齿顶边缘。"""
    candidates=[]
    for k,rotation in enumerate(result.rotations):
        grid=rotation.grid
        for tooth in np.unique(grid.tooth_pair_index):
            mask=grid.tooth_pair_index==tooth
            pressure=np.asarray(rotation.pressure_pa)[mask]
            b=np.asarray(rotation.half_width_m)[mask]
            if not np.any(pressure>0):continue
            inside=True
            for role in ('pinion','wheel'):
                gear=getattr(result.pair,role);low,high=gear.involute_roll_bounds_m
                arc=np.asarray(getattr(grid,'roll_'+role+'_m'))[mask]**2/(2*gear.base_radius_m)
                inside &= bool(np.all(arc[pressure>0]-b[pressure>0]>=low**2/(2*gear.base_radius_m)) and
                               np.all(arc[pressure>0]+b[pressure>0]<=high**2/(2*gear.base_radius_m)))
            if inside:candidates.append((float(np.max(pressure)),k,int(tooth)))
    if not candidates:raise ValueError('没有两轮Hertz带均位于有效齿面内部的有载样本')
    _,phase,tooth=max(candidates)
    return phase,tooth


def build_contact_pattern(result, *, member='pinion', profile_points=1601, snapshot_index=None,
                          snapshot_tooth_pair_index=None):
    """兼容旧有限带算法：把p0、b映射至齿面并取包络，不等同于LTCA显示面。

    仅支持无延伸的直齿内/外啮合，条带须覆盖共同齿宽且等分。横向为原
    条带分片常值，不对齿宽或相位做平滑。沿齿廓采用精确渐开线弧长
    s=ρ²/(2rb)，局部压力p=p0 sqrt(1-((s-s0)/b)²)。未覆盖材料保持零；
    相邻正载样本Hertz带之间的空隙另存unresolved_sampling_mask，不能
    把显示空隙自动解释为实际脱离接触。实际齿面以有效形径裁剪。
    """
    if member not in ('pinion', 'wheel'):
        raise ValueError('member必须为pinion或wheel')
    _integer(profile_points, '齿廓显示采样数', 51)
    pair=result.pair
    if any(getattr(result,'inputs',{}).get(key,False) for key in ('external_boundary_gap','external_compliance_provider')):
        raise ValueError('外部回调可能包含逐齿非周期差异，当前接口不能据此折叠成相同齿通过程')
    if abs(pair.pinion.helix_angle_rad)>1e-14 or abs(pair.wheel.helix_angle_rad)>1e-14:
        raise ValueError('当前完整斑点后处理仅实现直齿；斜齿须另作接触线方向映射')
    if result.summary.get('extended_contact', False):
        raise ValueError('当前斑点不支持齿顶延伸；不能把延伸域当完整渐开线')
    phases=np.asarray(result.phase, dtype=float)
    if phases.ndim!=1 or len(phases)<2 or len(result.rotations)!=len(phases) or np.any(~np.isfinite(phases)):
        raise ValueError('需要完整有限单周期相位及逐相位原数组')
    ordered=np.sort(phases%1)
    spacing=np.diff(np.r_[ordered,ordered[0]+1])
    if not np.allclose(spacing, 1/len(phases), rtol=1e-9, atol=1e-12):
        raise ValueError('需要均匀完整基节周期；不允许以缺相位数据拼造完整通过程')
    if snapshot_index is None:
        snapshot_index=max(range(len(phases)),key=lambda j:float(np.max(result.rotations[j].pressure_pa)))
    _integer(snapshot_index,'瞬时相位序号',0)
    if snapshot_index>=len(phases):raise ValueError('瞬时相位序号越界')
    rotation=result.rotations[snapshot_index]
    selected_id=int(rotation.grid.tooth_pair_index[int(np.argmax(rotation.pressure_pa))]) if snapshot_tooth_pair_index is None else snapshot_tooth_pair_index
    if isinstance(selected_id,bool) or not isinstance(selected_id,(int,np.integer)) or selected_id not in rotation.grid.tooth_pair_index:
        raise ValueError('瞬时齿对序号不在所选相位的原始网格中')
    fields={key:[] for key in ('phase_index','phase','tooth_pair_index','face_m','roll_pinion_m',
          'roll_wheel_m','strip_width_m','force_n','pressure_pa','half_width_m')}
    lines=[]; reference_face=None; reference_width=None
    force_errors=[]
    for k,rotation in enumerate(result.rotations):
        grid=rotation.grid
        for tooth in np.unique(grid.tooth_pair_index):
            ids=np.flatnonzero(grid.tooth_pair_index==tooth)
            ids=ids[np.argsort(np.asarray(grid.face_coordinate_m)[ids])]
            x=np.asarray(grid.face_coordinate_m)[ids]
            width=np.asarray(grid.widths_m)[ids]
            rho=np.asarray(getattr(grid,'roll_'+member+'_m'))[ids]
            pressure=np.asarray(rotation.pressure_pa)[ids]
            half=np.asarray(rotation.half_width_m)[ids]
            force=np.asarray(rotation.solution.forces_n)[ids]
            if any(np.any(~np.isfinite(v)) for v in (x,width,rho,pressure,half,force)) or np.any(width<=0) or np.any(pressure<0) or np.any(half<0) or np.any(force<0):
                raise ValueError('原接触数组须有限，条带长度须正，压力/半宽/力须非负')
            if not np.allclose(rho,rho[0],rtol=0,atol=1e-12):
                raise ValueError('直齿接触线应具有同一材料展长')
            if reference_face is None:
                reference_face=x.copy();reference_width=width.copy()
                left,right=pair.face_bounds_m
                expected=left+(np.arange(len(x))+.5)*(right-left)/len(x)
                if not np.allclose(x,expected,rtol=0,atol=1e-12) or not np.allclose(width,(right-left)/len(x),rtol=1e-9,atol=1e-12):
                    raise ValueError('当前斑点接口要求覆盖共同齿宽的等分条带')
            elif len(x)!=len(reference_face) or not np.allclose(x,reference_face,rtol=0,atol=1e-12) or not np.allclose(width,reference_width,rtol=1e-9,atol=1e-12):
                raise ValueError('各接触线须使用同一齿宽条带布局')
            recovered=math.pi/2*pressure*half*width
            force_errors.extend(np.abs(recovered-force))
            if not np.allclose(recovered,force,rtol=2e-9,atol=1e-10):
                raise ValueError('原p0、b、条带力不满足Hertz线接触积分，不能绘制守恒接触带')
            lines.append(dict(rho=float(rho[0]),p=pressure,b=half,k=k,tooth=int(tooth)))
            for key,value in dict(phase_index=np.full(len(ids),k),phase=np.full(len(ids),phases[k]),
                 tooth_pair_index=np.full(len(ids),tooth),face_m=x,roll_pinion_m=np.asarray(grid.roll_pinion_m)[ids],
                 roll_wheel_m=np.asarray(grid.roll_wheel_m)[ids],strip_width_m=width,force_n=force,
                 pressure_pa=pressure,half_width_m=half).items():fields[key].append(value)
    gear=getattr(pair,member);rb=gear.base_radius_m
    low,high=gear.involute_roll_bounds_m
    rho_edges=np.linspace(low,high,profile_points+1)
    rho_axis=(rho_edges[:-1]+rho_edges[1:])/2
    arc_axis=rho_axis**2/(2*rb)
    sweep=np.zeros((profile_points,len(reference_face)))
    instant=np.zeros_like(sweep);unresolved=np.zeros_like(sweep,dtype=bool)
    lines.sort(key=lambda row:row['rho'])
    for row in lines:
        centre=row['rho']**2/(2*rb)
        maximum=float(np.max(row['b']))
        if maximum==0:continue
        begin=np.searchsorted(arc_axis,centre-maximum,'left')
        end=np.searchsorted(arc_axis,centre+maximum,'right')
        delta=arc_axis[begin:end,None]-centre
        relative=np.divide(delta,row['b'][None,:],out=np.full((end-begin,len(reference_face)),np.inf),where=row['b'][None,:]>0)
        pressure=row['p'][None,:]*np.sqrt(np.maximum(0,1-relative**2))
        sweep[begin:end]=np.maximum(sweep[begin:end],pressure)
    centres=np.array([row['rho']**2/(2*rb) for row in lines])
    if np.any(np.diff(centres)<1e-16):raise ValueError('材料展长样本重复，不能确定完整通过程索引')
    unresolved_length=0.0
    # 只诊断相邻正载样本之间且未被任何已解Hertz带覆盖的空隙，不冒称全部未知域。
    for col in range(len(reference_face)):
        intervals=sorted((centre-row['b'][col],centre+row['b'][col])
                         for centre,row in zip(centres,lines) if row['p'][col]>0)
        merged=[]
        for begin,end in intervals:
            if merged and begin<=merged[-1][1]:merged[-1][1]=max(merged[-1][1],end)
            else:merged.append([begin,end])
        for j,(before,after) in enumerate(zip(lines[:-1],lines[1:])):
            begin,end=centres[j]+before['b'][col],centres[j+1]-after['b'][col]
            if before['p'][col]<=0 or after['p'][col]<=0 or end<=begin:continue
            cursor=begin
            for covered_begin,covered_end in merged:
                if covered_end<=cursor:continue
                if covered_begin>=end:break
                if covered_begin>cursor:
                    stop=min(covered_begin,end)
                    unresolved_length+=(stop-cursor)*reference_width[col]
                    unresolved[:,col]|=(arc_axis>cursor)&(arc_axis<stop)
                cursor=max(cursor,covered_end)
                if cursor>=end:break
            if cursor<end:
                unresolved_length+=(end-cursor)*reference_width[col]
                unresolved[:,col]|=(arc_axis>cursor)&(arc_axis<end)
    unresolved &= sweep<=0
    path_low,path_high=pair.contact_roll_bounds_m
    if member=='wheel':
        span=pair.center_distance_m*math.sin(pair.working_pressure_angle_rad)
        path_low,path_high=sorted((span+path_low,span+path_high) if pair.wheel.internal else (span-path_high,span-path_low))
    theoretical=(rho_axis>=path_low)&(rho_axis<=path_high)
    area=(path_high**2-path_low**2)/(2*rb)*pair.face_overlap_m
    original_peak=max(float(np.max(row['p'])) for row in lines)
    arrays={key:np.concatenate(value) for key,value in fields.items()}
    instant_force=float(np.sum(arrays['force_n'][arrays['phase_index']==snapshot_index]))
    selected=(arrays['phase_index']==snapshot_index)&(arrays['tooth_pair_index']==selected_id)
    s_low,s_high=low**2/(2*rb),high**2/(2*rb)
    selected_b=arrays['half_width_m'][selected]
    selected_p=arrays['pressure_pa'][selected]
    selected_rho=arrays['roll_'+member+'_m'][selected]
    selected_width=arrays['strip_width_m'][selected]
    centre=selected_rho**2/(2*rb)
    # 瞬时窄带使用自身弧长网格，避免全齿面均匀像素在低载时漏掉大部分接触力。
    if np.any(selected_b>0):
        snapshot_edges_s=np.linspace(max(s_low,float(np.min(centre-selected_b))),
                                     min(s_high,float(np.max(centre+selected_b))),profile_points+1)
        snapshot_axis_s=(snapshot_edges_s[:-1]+snapshot_edges_s[1:])/2
        ratio=np.divide(snapshot_axis_s[:,None]-centre[None,:],selected_b[None,:],
            out=np.full((profile_points,len(selected_b)),np.inf),where=selected_b[None,:]>0)
        instant=selected_p[None,:]*np.sqrt(np.maximum(0,1-ratio*ratio))
    else:
        snapshot_edges_s=np.linspace(s_low,s_high,profile_points+1)
        snapshot_axis_s=(snapshot_edges_s[:-1]+snapshot_edges_s[1:])/2
        instant=np.zeros_like(sweep)
    def primitive(u):return (u*np.sqrt(np.maximum(0,1-u*u))+np.arcsin(u))/2
    lower=np.divide(s_low-centre,selected_b,out=np.full_like(centre,-np.inf),where=selected_b>0)
    upper=np.divide(s_high-centre,selected_b,out=np.full_like(centre,np.inf),where=selected_b>0)
    clipped_force=float(np.sum(selected_p*selected_b*selected_width*(primitive(np.clip(upper,-1,1))-primitive(np.clip(lower,-1,1)))))
    areas=np.diff(snapshot_edges_s)[:,None]*reference_width[None,:]
    raster_force=float(np.sum(instant*areas))
    arrays.update(face_edges_m=np.linspace(*pair.face_bounds_m,len(reference_face)+1),
        profile_roll_edges_m=rho_edges,profile_roll_m=rho_axis,
        profile_arc_m=arc_axis,sweep_pressure_pa=sweep,instant_pressure_pa=instant,
        snapshot_profile_roll_edges_m=np.sqrt(2*rb*snapshot_edges_s),
        snapshot_profile_arc_m=snapshot_axis_s,
        unresolved_sampling_mask=unresolved,theoretical_path_mask=theoretical)
    summary={'member':member,'phases':len(phases),'strips':len(reference_face),'profile_points':profile_points,
       'source_nodes':len(arrays['force_n']),'source_contact_lines':len(lines),
       'snapshot_index':snapshot_index,'snapshot_phase':float(phases[snapshot_index]),
       'snapshot_tooth_pair_index':selected_id,'snapshot_all_pair_force_n':instant_force,
       'snapshot_selected_pair_force_n':float(np.sum(arrays['force_n'][selected])),
       'snapshot_analytical_clipped_force_n':clipped_force,
       'snapshot_grid_integrated_force_n':raster_force,
       'snapshot_grid_force_relative_error':abs(raster_force-clipped_force)/clipped_force if clipped_force>0 else None,
       'source_peak_pressure_pa':original_peak,'map_peak_pressure_pa':float(np.max(sweep)),
       'hertz_strip_force_max_absolute_error_n':max(force_errors),
       'unresolved_between_loaded_samples_area_m2':unresolved_length,
       'unresolved_between_loaded_samples_fraction':unresolved_length/area,
       'source_normal_force_n':result.summary['normal_force_n'],
       'source_torque_n_m':result.summary['torque_n_m'],
       'theoretical_contact_roll_bounds_m':[path_low,path_high],
       'full_effective_flank_roll_bounds_m':[low,high],
       'scope':'直齿局部Hertz带、周期相同齿等价拼接的已采样相位最大值；非二维FE接触或实测斑点',
       'interpolated_pressure':False,'physical_validation':False}
    return ContactPattern(arrays,summary)


def export_contact_pattern(pattern, output_dir):
    """保存SI斑点矩阵、原条带样本与单位/哈希，NPZ不使用对象或pickle。"""
    path=Path(output_dir);path.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(path/'pattern.npz',**pattern.arrays)
    write_json(path/'summary.json',pattern.summary)
    write_json(path/'manifest.json',{'units':{'length':'m','pressure':'Pa','force':'N'},
        'layout':'sweep压力用profile_roll_edges_m；instant压力用snapshot_profile_roll_edges_m；横向均为face_edges_m相邻条带。原采样以phase_index/tooth_pair_index区分',
        'instantaneous':'只显示snapshot_index及snapshot_tooth_pair_index的一对齿',
        'sweep':'所有相位与周期等价相邻齿对的有限Hertz带最大值，不在空隙平滑填色',
        'files':{name:hashlib.sha256((path/name).read_bytes()).hexdigest() for name in ('pattern.npz','summary.json')}})
