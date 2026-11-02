"""新写接触图纠错证据：受控输入比较与既有18/77显示参照复算。

修形候选只用于诊断；不替换原关节基准，也不更新系统分载或宣称制造最优。
"""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import numpy as np

from gear_contact import analyze_config
from gear_contact.io import export_result, write_json
from robot_joint.contact_display import contact_pass_data, draw_contact_pass
from robot_joint.figure_style import publication_settings, save_publication_figure, COLORS


def line_forces(result):
    """一个基节周期的相邻齿对折叠至同一材料通过程；合力始终来自原条带。"""
    records=[]
    for rotation in result.rotations:
        for tooth in np.unique(rotation.grid.tooth_pair_index):
            mask=rotation.grid.tooth_pair_index==tooth
            records.append((float(np.mean(rotation.grid.roll_pinion_m[mask])),
                            float(np.sum(rotation.solution.forces_n[mask]))))
    return np.asarray(sorted(records))


def _calculate(output):
    bundle=json.loads((output/'inputs.json').read_text(encoding='utf-8'))
    case=next(c for c in bundle['cases'] if c['name']=='negative_peak')
    choices=[c for c in case['meshes'] if c['mesh']=='sun_planet']
    item=next((c for c in choices if c['planet']==2),choices[0])
    baseline=deepcopy(item['analysis'])
    baseline.update(phases=128,strips=21,elastic_mode='iso',local_contact=False)
    love=deepcopy(baseline);love['elastic_mode']='iso_love'
    crown=deepcopy(love)
    for role in ('modification_pinion','modification_wheel'):
        if crown.get(role,{}).get('kind','parameters')!='parameters':
            raise ValueError('修形诊断只支持显式参数修形，不能覆盖组合修形')
        crown.setdefault(role,{})['profile_crown_m']=3e-6
    arrays={};records=[]
    for index,(name,config) in enumerate(zip(('iso_baseline','love_only','love_crown_each_3um'),(baseline,love,crown))):
        result=analyze_config(config)
        export_result(result,output/'comparison_raw'/name)
        data=contact_pass_data(result)
        for key in ('face_coordinate_m','roll_distance_m','pressure_pa_x_y'):
            arrays[f'control_{index}_{key}']=data[key]
        force=line_forces(result)
        arrays[f'control_{index}_line_roll_force']=force
        arrays[f'control_{index}_phase']=result.phase
        arrays[f'control_{index}_approach_m']=result.normal_approach_m
        records.append(dict(name=name,input=config,source_summary=result.summary,
            peak_pressure_pa=float(data['samples'][:,2].max()),
            mean_approach_m=float(np.mean(result.normal_approach_m)),
            max_neighbour_force_jump_over_total=float(np.max(np.abs(np.diff(force[:,1])))/result.summary['normal_force_n']),
            uncovered_display_count=int(np.count_nonzero(~np.isfinite(data['pressure_pa_x_y'])))))
    changes={key:{'source':item['analysis'].get(key),'control':baseline[key]}
        for key in ('phases','strips','elastic_mode','local_contact') if item['analysis'].get(key)!=baseline[key]}
    model=dict(case=case['name'],mesh='sun_planet',planet=item['planet'],phases=128,strips=21,
        source_analysis_input=item['analysis'],baseline_changes_from_source=changes,
        candidate_profile_rule='replace_each_member_with_3um',
        records=records,system_load_sharing_recomputed=False,physical_validation=False,
        scope_zh='指定ISO控制基准、仅切换Love、Love下每轮齿廓鼓形替换3µm；原输入和所有控制改动明确保存。三组实际重算，候选非最优、非制造推荐；不能外推总成。')
    write_json(output/'model_comparison.json',model)
    root=next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    fixture=root/'benchmarks/reference/contact_display_18_77'
    provenance=json.loads((fixture/'provenance.json').read_text(encoding='utf-8'))
    for name,source in provenance['files'].items():
        if hashlib.sha256((fixture/name).read_bytes()).hexdigest()!=source['sha256']:
            raise ValueError('18/77显示参照身份不匹配：'+name)
    config=json.loads((fixture/'analysis_input.json').read_text(encoding='utf-8'))
    result=analyze_config(config)
    export_result(result,output/'comparison_raw'/'helical_18_77')
    with np.load(fixture/'native_display.npz',allow_pickle=False) as source:
        data=contact_pass_data(result,face_coordinate_m=source['face_coordinate_m'],roll_distance_m=source['roll_distance_m'])
        native=source['reference_pressure_pa_x_y'].copy()
        upstream=source['project_pressure_pa_x_y'].copy()
        for key in ('face_coordinate_m','roll_distance_m','pressure_pa_x_y'):
            arrays['reference_'+key]=data[key]
    arrays['reference_native_pressure_pa_x_y']=native
    arrays['reference_upstream_pressure_pa_x_y']=upstream
    same=np.isfinite(upstream)&np.isfinite(data['pressure_pa_x_y'])
    common=np.isfinite(native)&np.isfinite(data['pressure_pa_x_y'])
    difference=data['pressure_pa_x_y'][common]-native[common]
    metrics=dict(upstream_commit=provenance['upstream_commit'],grid_shape_x_y=list(native.shape),
        upstream_display_max_abs_difference_pa=float(np.max(np.abs(data['pressure_pa_x_y'][same]-upstream[same]))),
        upstream_nan_mask_equal=bool(np.array_equal(np.isnan(upstream),np.isnan(data['pressure_pa_x_y']))),
        masta_common_finite_count=int(np.count_nonzero(common)),
        masta_display_mae_pa=float(np.mean(np.abs(difference))),
        masta_display_max_abs_difference_pa=float(np.max(np.abs(difference))),
        masta_display_rmse_pa=float(np.sqrt(np.mean(difference**2))),
        masta_display_peak_pa=float(np.nanmax(native)),project_display_peak_pa=float(np.nanmax(data['pressure_pa_x_y'])),
        project_raw_endpoint_peak_pa=float(data['samples'][:,2].max()),
        project_uncovered_count=int(np.count_nonzero(~np.isfinite(data['pressure_pa_x_y']))),
        masta_run_in_this_revision=False,assembly_validation=False,
        scope_zh='既有MASTA30×30显示面离线比较；与原端点压力的误差指标不同。上游显示复现误差与MASTA差异分开报告。')
    write_json(output/'reference_reproduction.json',metrics)
    np.savez_compressed(output/'comparison_data.npz',**arrays)
    names=('model_comparison.json','reference_reproduction.json','comparison_data.npz')
    write_json(output/'comparison_manifest.json',dict(files={name:hashlib.sha256((output/name).read_bytes()).hexdigest() for name in names}))


def build_comparisons(output, *, languages=('zh','en'), replot=False):
    """计算后保存可校验绘图数组；重绘只读取既有响应，不调用求解器。"""
    output=Path(output)
    if not replot:
        _calculate(output)
    manifest=json.loads((output/'comparison_manifest.json').read_text(encoding='utf-8'))
    for name,expected in manifest['files'].items():
        if hashlib.sha256((output/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('比较图数据身份不匹配：'+name)
    model=json.loads((output/'model_comparison.json').read_text(encoding='utf-8'))
    metrics=json.loads((output/'reference_reproduction.json').read_text(encoding='utf-8'))
    with np.load(output/'comparison_data.npz',allow_pickle=False) as source:
        arrays={key:source[key].copy() for key in source.files}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for language in languages:
        L=lambda zh,en:zh if language=='zh' else en
        def finish(fig,name,title,note):
            save_publication_figure(fig,output/language/name,title=title,footnote=note,language=language)
            plt.close(fig)
        with plt.rc_context(publication_settings(language)):
            fig,axes=plt.subplots(2,3,figsize=(13,9),layout='constrained')
            maximum=max(r['peak_pressure_pa'] for r in model['records'])
            titles=(L('ISO控制基准','ISO control baseline'),L('仅切换Love柔度','Love compliance only'),L('Love + 每轮3 μm齿廓鼓形','Love + 3 μm profile crown/member'))
            for index,record in enumerate(model['records']):
                data={key:arrays[f'control_{index}_{key}'] for key in ('face_coordinate_m','roll_distance_m','pressure_pa_x_y')}
                data['interpolated']=True
                artist=draw_contact_pass(axes[0,index],data,maximum_pa=maximum,language=language,
                    title=titles[index]+f"\n{record['peak_pressure_pa']/1e6:.1f} MPa")
                fig.colorbar(artist,ax=axes[0,index],label='MPa',shrink=.75)
                force=arrays[f'control_{index}_line_roll_force']
                axes[1,index].plot(force[:,0]*1e3,force[:,1],color=COLORS[0 if index<2 else 1])
                axes[1,index].set(xlabel=L('小轮展长 / mm','Pinion roll / mm'),ylabel=L('单齿对法向合力 / N','One-pair normal force / N'),
                    title=L('平均接近量 ','Mean approach ')+f"{record['mean_approach_m']*1e6:.3f} μm",
                    ylim=(0,model['records'][0]['source_summary']['normal_force_n']*1.07))
                axes[1,index].grid(alpha=.2)
            finish(fig,'07_model_comparison',L('受控重算：原分载跳变与显式修形候选','Controlled solves: baseline load jumps and an explicit modification candidate'),
                L('反向峰载太阳副行星2，128×21；候选不是最优修形；载荷固定，未反馈系统；压力没有按外观修改。',
                  'Negative peak, S–P planet 2, 128×21; candidate is not optimized; fixed local load, no system feedback or pressure reshaping.'))
            fig,axes=plt.subplots(1,2,figsize=(12,6.3),layout='constrained')
            maximum=max(metrics['masta_display_peak_pa'],metrics['project_display_peak_pa'])
            for index,name in enumerate(('reference_native_pressure_pa_x_y','reference_pressure_pa_x_y')):
                data={key:arrays['reference_'+key] for key in ('face_coordinate_m','roll_distance_m')}
                data.update(pressure_pa_x_y=arrays[name],interpolated=True)
                artist=draw_contact_pass(axes[index],data,maximum_pa=maximum,language=language,
                    title=L('MASTA：既有30×30显示面','MASTA: saved 30×30 display') if index==0 else L('本次独立重算：相同网格','Independent recomputation: same grid'))
                fig.colorbar(artist,ax=axes[index],label='MPa',shrink=.8)
            finish(fig,'08_reference_reproduction',L('18/77斜齿副：通过程接触图离线核对','18/77 helical mesh: offline tooth-passage display comparison'),
                L('同一输入；源项目显示再现误差0 Pa；与MASTA差异另计；本次未运行MASTA，不代表关节总成验证。',
                  'Same input; upstream display reproduced (0 Pa); MASTA differences reported separately; no new MASTA run or joint validation.'))


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description='接触图受控重算及18/77显示参照核对')
    parser.add_argument('--output',type=Path,default=Path('examples/results/contact_patterns'))
    parser.add_argument('--language',choices=('zh','en','both'),default='both')
    parser.add_argument('--replot',action='store_true')
    args=parser.parse_args()
    build_comparisons(args.output,languages=('zh','en') if args.language=='both' else (args.language,),replot=args.replot)
