"""复算实物需求参照案例，并由同一结果生成中英对应工程图表与数据。"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np

from robot_joint import analyze_joint, thermal_response, reversal_curve, sn_damage
from robot_joint.io import export_joint
from gear_contact.io import write_json
from .pendulum_case import pendulum_duty, candidate_configs, validate_saved_duty

COLORS = ['#155f78','#de8d35','#7482a4','#318a6e']
FIGURES = ['01_design','02_duty','03_load_sharing','04_contact','05_strength_support',
           '06_precision','07_motor_thermal','08_fatigue','09_comparison']


def project_dir():
    """按独立项目标记定位，允许从任意工作目录调用。"""
    return next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())


def dense_motion(requirements, samples=1001):
    """用于展示的连续五次轨迹；计算工况依然使用公开保存的中点离散。"""
    p, d = requirements['pendulum'], requirements['motion_design']
    period = 2*(d['move_duration_s']+d['hold_duration_s'])
    t = np.linspace(0,period,samples)
    q, w, a = np.zeros_like(t),np.zeros_like(t),np.zeros_like(t)
    half = period/2
    for j, time in enumerate(t):
        reverse = time >= half
        local = time-half if reverse else time
        start,end = (d['amplitude_rad'],-d['amplitude_rad']) if reverse else (-d['amplitude_rad'],d['amplitude_rad'])
        if local >= d['move_duration_s']:
            q[j] = end
            continue
        u=local/d['move_duration_s']
        q[j]=start+(end-start)*(10*u**3-15*u**4+6*u**5)
        w[j]=(end-start)*(30*u**2-60*u**3+30*u**4)/d['move_duration_s']
        a[j]=(end-start)*(60*u-180*u**2+120*u**3)/d['move_duration_s']**2
    inertia=(p['point_mass_kg']+p['rod_mass_kg']/3)*p['length_m']**2
    moment=(p['point_mass_kg']+p['rod_mass_kg']/2)*p['length_m']
    torque=inertia*a+p['gravity_m_per_s2']*moment*np.sin(q)+d['viscous_damping_n_m_s_per_rad']*w
    return t,q,w,a,torque


def prepare_data(base, requirements, results):
    """汇总报告指标，所有双语图表共用这一份数值和原接触对象。"""
    result=results[0]
    _,derived=pendulum_duty(requirements)
    time=np.array([s['time_s'] for s in derived['states']])
    peak=max(range(len(result.cases)),key=lambda j:max(a.summary['max_pressure_pa'] for group in result.cases[j]['contacts'].values() for a in group))
    contacts={key:max(group,key=lambda a:a.summary['max_pressure_pa']) for key,group in result.cases[peak]['contacts'].items()}
    cycles=sn_damage(result.fatigue['cycles'],**{k:v for k,v in base['fatigue'].items() if k!='stress_history_pa'})
    losses=[{key:s[key] for key in ('duration_s','motor_loss_w','gear_loss_w')} for s in result.motor['segments']]
    cold=thermal_response(losses*100,**base['thermal'])
    study=[]
    for name,r in zip(['baseline','larger_errors','flexible_pins'],results):
        row={'candidate':name,**r.summary}
        row['pareto_nondominated']=r.summary['configured_checks_pass'] and not any(
            other.summary['configured_checks_pass'] and all(other.summary[k]<=r.summary[k] for k in
              ('maximum_contact_pressure_pa','maximum_angle_budget_rad','maximum_motor_temperature_k'))
            and any(other.summary[k]<r.summary[k] for k in
              ('maximum_contact_pressure_pa','maximum_angle_budget_rad','maximum_motor_temperature_k'))
            for other in results if other is not r)
        study.append(row)
    t,q,w,a,tau=dense_motion(requirements)
    dense_peak=float(np.max(np.abs(tau)))
    midpoint_peak=max(abs(s['output_torque_n_m']) for s in base['duty_segments'])
    metrics={'case_id':base['case_id'],'derived':derived,'summary':result.summary,
      'peak_case_index':peak,'midpoint_peak_torque_n_m':midpoint_peak,'dense_peak_torque_n_m':dense_peak,
      'torque_peak_discretization_relative_difference':abs(midpoint_peak-dense_peak)/dense_peak,
      'motor_rms_current_a':result.motor['rms_current_a'],'fatigue_damage_per_input_history':cycles['damage_per_duty'],
      'thermal_periodic_residual_k':result.thermal['periodic_residual_k'],
      'maximum_path_residual_m':max(c['sharing']['coupling']['path_residual_m'] for c in result.cases),
      'minimum_active_planets':min(c['sharing']['diagnostics']['active_planets'] for c in result.cases),
      'candidate_comparison':study,'figures':FIGURES,'languages':['zh','en'],
      'representative_meshes':{key:contact.summary for key,contact in contacts.items()},
      'model_scope_zh':requirements['case_identity']['scope_zh'],
      'model_scope_en':requirements['case_identity']['scope_en']}
    return dict(base=base,requirements=requirements,result=result,results=results,time=time,derived=derived,
                peak=peak,contacts=contacts,cycles=cycles,cold=cold,metrics=metrics,dense=(t,q,w,a,tau))


def render_figures(data, output, language):
    """分别渲染中文和英文标注，不在英文图中嵌入中文工况名称。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.patches import Circle
    names={f.name for f in font_manager.fontManager.ttflist}
    chinese=next((name for name in ('Microsoft YaHei','SimHei','Noto Sans CJK SC') if name in names),'DejaVu Sans')
    L=lambda zh,en:zh if language=='zh' else en
    base,req,r=data['base'],data['requirements'],data['result']
    time=data['time']; count=len(r.cases); n=base['stage']['planet_count']
    path=Path(output)/language; path.mkdir(parents=True,exist_ok=True)
    def style(ax):
        ax.grid(alpha=.14); ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
    def save(fig,name,title):
        fig.suptitle(title,fontsize=15,fontweight='bold')
        # 为底部图义保留独立空间，避免与横轴标题相互覆盖。
        fig.get_layout_engine().set(rect=(0,.055,1,.93))
        fig.text(.012,.008,L('真实台架需求参照 · 新设计模型计算，非样机实测','Physical-testbed requirements · redesign-model results, not measurements'),
                 fontsize=8,color='#677681')
        fig.savefig(path/(name+'.png'),dpi=160,facecolor='white')
        fig.savefig(path/(name+'.svg'),facecolor='white',metadata={'Title':title,'Description':'language='+language})
        plt.close(fig)
    settings={'font.family':chinese if language=='zh' else 'DejaVu Sans','font.size':10,
              'axes.unicode_minus':False,'axes.titlepad':12,'axes.labelpad':7,
              'legend.frameon':False,'axes.spines.top':False,'axes.spines.right':False}
    with plt.rc_context(settings):
        fig,axes=plt.subplots(1,2,figsize=(12,5.8),layout='constrained')
        ax=axes[0];theta=math.pi/3;length=.5
        x,y=length*math.sin(theta),-length*math.cos(theta)
        ax.plot([0,x],[0,y],lw=7,color=COLORS[0],solid_capstyle='round')
        ax.add_patch(Circle((x,y),.035,color=COLORS[1]));ax.add_patch(Circle((0,0),.025,color='#243c48'))
        ax.plot([0,0],[0,-.55],'--',color='#9ca8af',lw=1)
        ax.annotate(L('点质量 0.546 kg','Point mass 0.546 kg'),(x,y),xytext=(.13,-.48),arrowprops={'arrowstyle':'-','color':'#7c8a93'})
        ax.text(.02,.07,L('关节轴 z / 输出端','Joint axis z / output'),fontweight='bold')
        ax.text(-.22,-.22,L('杆件 0.130 kg\n长度 0.5 m','Rod 0.130 kg\nLength 0.5 m'))
        ax.text(-.24,-.56,L('新设计运动：±60°往返及保持','Designed duty: ±60° sweeps and holds'))
        ax.set(xlim=(-.27,.60),ylim=(-.62,.15),aspect='equal',title=L('公开实物台架需求','Published physical-testbed requirements'));ax.axis('off')
        ax=axes[1];g=r.geometry;scale=1000
        for radius,ls,lw in [(27.5,'-',2),(25,'--',1),(24.5,'-',.7),(25.625,'-',.7)]:ax.add_patch(Circle((0,0),radius,fill=False,color=COLORS[0],lw=lw,ls=ls))
        ax.add_patch(Circle((0,0),5,fill=False,color=COLORS[1],lw=2));ax.add_patch(Circle((0,0),15,fill=False,color=COLORS[3],ls=':',lw=1))
        for j,(px,py) in enumerate(g['planet_centres_xy_m']*scale):
            ax.add_patch(Circle((px,py),10,fill=False,color=COLORS[2],lw=1.7))
            ax.plot(px,py,'+',color=COLORS[3]);ax.text(px,py,str(j+1),ha='center',va='bottom',fontsize=11)
        ax.text(0,0,L('太阳轮\n20齿','Sun\n20T'),ha='center',va='center',fontsize=9)
        ax.set(xlim=(-35,35),ylim=(-35,35),aspect='equal',xlabel='mm',ylabel='mm',
               title=L('公开齿轮组几何 → 6:1布局','Published gearset geometry → 6:1 layout'))
        ax.text(0,-33,L('m=0.5 mm · 20/40/100齿 · 有效齿宽5 mm','m=0.5 mm · 20/40/100T · effective width 5 mm'),ha='center',fontsize=9)
        ax.text(0,31,L('参考圆布局；不是真实齿廓','Reference-circle layout; not manufactured tooth profiles'),ha='center',fontsize=8,color='#677681')
        save(fig,FIGURES[0],L('6:1摆关节再设计：从实物需求到参数化模型','A 6:1 pendulum-joint redesign: physical requirements to a model'))

        t,q,w,a,tau=data['dense'];fig,axes=plt.subplots(3,1,figsize=(11.5,8.5),sharex=True,layout='constrained')
        for ax,curve,label,color in zip(axes,[np.degrees(q),tau,w],
            [L('角度 / °','Angle / deg'),L('输出扭矩 / N·m','Output torque / N·m'),L('角速度 / rad/s','Angular speed / rad/s')],COLORS):
            ax.plot(t,curve,color=color,lw=2);ax.set_ylabel(label);style(ax)
        axes[1].scatter(time,[s['output_torque_n_m'] for s in base['duty_segments']],color=COLORS[1],s=24,zorder=3,label=L(f'{count}个代表工况',f'{count} representative states'))
        axes[1].legend(loc='upper right')
        axes[2].fill_between(t,0,w,where=tau*w<0,color=COLORS[1],alpha=.17,label=L('回驱区间 Tω<0','Backdriving, Tω<0'))
        axes[2].legend(loc='upper right');axes[2].set_xlabel(L('周期时间 / s','Cycle time / s'))
        save(fig,FIGURES[1],L('五次往返轨迹 → 惯性、重力与保持载荷','Quintic sweeps → inertial, gravitational and holding loads'))

        fig,axes=plt.subplots(2,1,figsize=(11.5,7),sharex=True,layout='constrained')
        forces=np.array([c['sharing']['tangential_forces_n'] for c in r.cases])
        for j in range(n):axes[0].plot(time,forces[:,j],'o-',color=COLORS[j],label=L(f'行星{j+1}',f'Planet {j+1}'),ms=4)
        axes[0].set_ylabel(L('单副切向力 / N','Tangential force per mesh / N'));axes[0].legend(ncol=3)
        axes[1].plot(time,[c['sharing']['load_share_factor'] for c in r.cases],'o-',color=COLORS[0],ms=4)
        axes[1].axhline(1,color='#87939c',ls='--',label=L('理想均载','Ideal sharing'))
        axes[1].set(xlabel=L('周期时间 / s','Cycle time / s'),ylabel=L('均载系数 Kγ','Load-sharing factor Kγ'))
        for ax in axes:style(ax)
        save(fig,FIGURES[2],L('误差与共享柔度决定各行星的实际载荷','Errors and shared compliance determine individual planet loads'))

        fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
        for col,(key,contact) in enumerate(data['contacts'].items()):
            label=L('太阳轮—行星轮','Sun–planet') if key=='sun_planet' else L('行星轮—内齿圈','Planet–ring')
            ax=axes[0,col];ax.plot(contact.phase,contact.normal_approach_m*1e6,'o-',color=COLORS[col],ms=4)
            ax.set(title=label,xlabel=L('啮合周期相位','Mesh-cycle phase'),ylabel=L('法向传动误差 / μm','Normal transmission error / μm'));style(ax)
            rotation=max(contact.rotations,key=lambda z:float(np.max(z.pressure_pa)))
            ax=axes[1,col];loaded=rotation.solution.active_mask
            sc=ax.scatter(rotation.grid.face_coordinate_m[loaded]*1e3,rotation.grid.roll_pinion_m[loaded]*1e3,
                 c=rotation.pressure_pa[loaded]/1e6,cmap='viridis',s=65,marker='s',vmin=0,vmax=r.summary['maximum_contact_pressure_pa']/1e6)
            ax.set(xlabel=L('齿宽位置 / mm','Face position / mm'),ylabel=L('小轮展长 / mm','Pinion roll length / mm'),
                   title=L('峰压相位：原始接触节点','Peak-pressure phase: raw contact nodes'))
            fig.colorbar(sc,ax=ax,label=L('Hertz峰压 / MPa','Hertz peak pressure / MPa'),shrink=.8);style(ax)
        save(fig,FIGURES[3],L('逐行星LTCA：传动误差与局部接触压力','Per-planet LTCA: transmission error and local contact pressure'))

        fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
        stress=[];labels=[]
        for key in ('sun_planet','planet_ring'):
            for role in ('pinion','wheel'):
                stress.append([max(g[role]['contact_stress_pa'] for c in r.cases for g in c['ratings'][key])/1e6,
                               max(g[role]['root_stress_pa'] for c in r.cases for g in c['ratings'][key])/1e6])
                labels.append({'sun_planet':L('太阳副','S–P'),'planet_ring':L('内齿副','P–R')}[key]+'\n'+L('小轮' if role=='pinion' else '大轮',role))
        x=np.arange(4);axes[0,0].bar(x-.17,np.array(stress)[:,0],width=.34,color=COLORS[0],label=L('评级接触应力','Rating contact stress'))
        axes[0,0].bar(x+.17,np.array(stress)[:,1],width=.34,color=COLORS[1],label=L('评级齿根应力','Rating root stress'))
        axes[0,0].set(xticks=x,xticklabels=labels,ylabel='MPa',title=L('显式因子标准应力','Explicit-factor rating stresses'));axes[0,0].legend(fontsize=8)
        for name,color,label in [('contact_safety_factor',COLORS[0],L('接触 SH','Contact SH')),('bending_safety_factor',COLORS[1],L('齿根 SF','Bending SF'))]:
            safety=[min(g[role][name] for group in c['ratings'].values() for g in group for role in ('pinion','wheel')) for c in r.cases]
            axes[0,1].plot(time,safety,'o-',color=color,label=label,ms=4)
        axes[0,1].axhline(1,color='#77838d',ls='--');axes[0,1].set(title=L('逐工况最小安全系数','Minimum safety factors by state'),xlabel=L('时间 / s','Time / s'));axes[0,1].legend()
        for j in range(2):axes[1,0].plot(time,[c['bearing_reactions']['radial_loads_n'][j] for c in r.cases],'o-',color=COLORS[j],label=L(f'支承{j+1}',f'Bearing {j+1}'),ms=4)
        axes[1,0].set(title=L('输出端双支承反力','Two output-support reactions'),xlabel=L('时间 / s','Time / s'),ylabel=L('径向载荷 / N','Radial load / N'));axes[1,0].legend()
        translations=[np.linalg.norm(c['support']['displacement_m'])*1e6 for c in r.cases]
        axes[1,1].plot(time,translations,'o-',color=COLORS[3],ms=4)
        axes[1,1].set(title=L('给定结构刚度下的平移量','Translation with the supplied structural stiffness'),xlabel=L('时间 / s','Time / s'),ylabel=L('平移范数 / μm','Translation norm / μm'))
        for ax in axes.flat:style(ax)
        save(fig,FIGURES[4],L('强度与支承：分别检查应力、裕量和结构变形','Strength and support: separate stress, margin and displacement checks'))

        fig,axes=plt.subplots(1,3,figsize=(14,5.2),layout='constrained')
        factor=180/math.pi*60; worst=max(r.cases,key=lambda c:c['precision']['deterministic_worst_case_error_rad']);p=worst['precision']
        parts=[abs(p['output_elastic_twist_rad']),p['backlash_half_width_rad'],p['encoder_output_half_count_rad'],abs(p['assembly_error_rad'])]
        names=[L('承载角','Loaded offset'),L('半侧隙','Half backlash'),L('半计数','Half count'),L('装配','Assembly')]
        axes[0].barh(names,np.array(parts)*factor,color=COLORS);axes[0].set(xlabel='arcmin',title=L('最坏工况的角度预算分量','Angle-budget terms at the worst state'))
        axes[1].plot(time,[c['precision']['output_torsional_stiffness_n_m_per_rad']/1000 for c in r.cases],'o-',color=COLORS[0],ms=4)
        axes[1].set(xlabel=L('时间 / s','Time / s'),ylabel='kN·m/rad',title=L('输出端串联等效刚度','Output stiffness with series structure'))
        torques=np.linspace(-6,6,101);curve=reversal_curve(torques,output_backlash_rad=p['output_backlash_rad'],torsional_stiffness_n_m_per_rad=r.summary['minimum_output_torsional_stiffness_n_m_per_rad'])
        for key,ls in [('positive_envelope_rad','-'),('negative_envelope_rad','--')]:axes[2].plot(torques,np.array(curve[key])*factor,ls,color=COLORS[0])
        axes[2].set(xlabel=L('输出扭矩 / N·m','Output torque / N·m'),ylabel='arcmin',title=L('双工作面静态包络\n非历史相关迟滞','Two-flank static envelope\nNot history-dependent hysteresis'))
        for ax in axes:style(ax)
        save(fig,FIGURES[5],L('精度：区分弹性、侧隙与编码器量化','Precision: separate elastic offset, backlash and encoder quantization'))

        fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained');m=base['motor'];speed=np.linspace(0,m['maximum_speed_rad_s'],200)
        for field,ls,label in [('continuous_torque_n_m','-',L('连续','Continuous')),('peak_torque_n_m','--',L('峰值','Peak'))]:
            axes[0,0].plot(speed,m[field]*np.minimum(1,m['base_speed_rad_s']/np.maximum(speed,1e-20)),ls,color=COLORS[0],label=label)
        for mode,mark,color,label in [('motoring','o',COLORS[0],L('驱动','Motoring')),('backdriving','s',COLORS[1],L('回驱','Backdriving')),('holding','^',COLORS[3],L('保持','Holding'))]:
            selected=[s for s in r.motor['segments'] if s['mode']==mode]
            axes[0,0].scatter([abs(s['motor_speed_rad_s']) for s in selected],[abs(s['motor_torque_n_m']) for s in selected],marker=mark,color=color,label=label,s=30,zorder=3)
        axes[0,0].set(xlabel=L('电机转速幅值 / rad/s','Motor speed magnitude / rad/s'),ylabel='N·m',title=L('电机包络与实际工况','Motor envelope and operating states'));axes[0,0].legend(fontsize=8,ncol=2)
        axes[0,1].plot(time,[s['copper_loss_w'] for s in r.motor['segments']],'o-',color=COLORS[1],label=L('铜损','Copper loss'),ms=4)
        axes[0,1].plot(time,[s['gear_loss_w'] for s in r.motor['segments']],'o-',color=COLORS[0],label=L('齿轮箱损耗','Gearbox loss'),ms=4)
        axes[0,1].set(xlabel=L('周期时间 / s','Cycle time / s'),ylabel='W',title=L('保持段仍有铜损','Holding still produces copper loss'));axes[0,1].legend()
        temp=np.array(data['cold']['temperature_k'])-273.15
        for j,label in enumerate([L('电机','Motor'),L('减速器','Gearbox')]):
            axes[1,0].plot(data['cold']['time_s'],temp[:,j],color=COLORS[j],label=label)
            axes[1,0].axhline(r.thermal['maximum_temperature_k'][j]-273.15,color=COLORS[j],ls='--',alpha=.7)
        axes[1,0].set(xlabel=L('冷启动后时间 / s','Time after cold start / s'),ylabel='°C',title=L('100次任务：向周期热稳态靠近','100 duties: approaching periodic steady state'));axes[1,0].legend()
        for j,label in enumerate([L('电机','Motor'),L('减速器','Gearbox')]):axes[1,1].plot(r.thermal['time_s'],np.array(r.thermal['temperature_k'])[:,j]-273.15,color=COLORS[j],label=label)
        axes[1,1].set(xlabel=L('周期时间 / s','Cycle time / s'),ylabel='°C',title=L('周期热稳态内的峰温','Temperatures within the periodic steady cycle'));axes[1,1].legend()
        for ax in axes.flat:style(ax)
        save(fig,FIGURES[6],L('机电与热：驱动、回驱、保持和重复任务','Motor and thermal checks: motoring, regeneration, holding and repetition'))

        fig,axes=plt.subplots(1,2,figsize=(12,5.5),layout='constrained')
        history=np.array(base['fatigue']['stress_history_pa'])/1e6
        axes[0].plot(np.arange(len(history)),history,'o-',color=COLORS[0]);axes[0].axhline(0,color='#8a959d',lw=.7)
        axes[0].set(xlabel=L('应力样本序号','Stress-sample index'),ylabel='MPa',title=L('显式假定热点映射：12 MPa/(N·m)','Assumed hotspot map: 12 MPa/(N·m)'))
        rows=data['cycles']['cycles'];den=sum(v['damage'] for v in rows)
        axes[1].bar(np.arange(1,len(rows)+1),[v['damage']/den*100 for v in rows],color=COLORS[1])
        axes[1].set(xlabel=L('雨流循环编号','Rainflow-cycle index'),ylabel=L('当前历程损伤占比 / %','Share of damage in this history / %'),title=L('给定S-N与Goodman修正后的Miner损伤','Miner damage with supplied S–N and Goodman correction'))
        for ax in axes:style(ax)
        save(fig,FIGURES[7],L('疲劳功能：输入应力历程 → 雨流 → 条件性损伤','Fatigue workflow: supplied stress history → rainflow → conditional damage'))

        fig,axes=plt.subplots(1,2,figsize=(12,5.5),layout='constrained')
        names=[L('基准','Baseline'),L('误差×3','Errors ×3'),L('销刚度×0.3','Pin stiffness ×0.3')]
        for j,(row,name) in enumerate(zip(data['metrics']['candidate_comparison'],names)):
            axes[0].scatter(row['maximum_angle_budget_rad']*factor,row['maximum_contact_pressure_pa']/1e6,s=90,color=COLORS[j])
            axes[0].annotate(name,(row['maximum_angle_budget_rad']*factor,row['maximum_contact_pressure_pa']/1e6),xytext=(6,7),textcoords='offset points',fontsize=9)
        axes[0].margins(x=.3,y=.3);axes[0].set(xlabel=L('最坏角度预算 / arcmin','Worst angle budget / arcmin'),ylabel=L('峰压 / MPa','Peak pressure / MPa'),title=L('两项指标均希望降低','Both objectives are minimized'))
        axes[1].bar(names,[row['minimum_output_torsional_stiffness_n_m_per_rad']/1000 for row in data['metrics']['candidate_comparison']],color=COLORS[:3])
        axes[1].set(ylabel='kN·m/rad',title=L('改善均载时也要关注刚度代价','Check stiffness cost when improving load sharing'))
        for ax in axes:style(ax)
        save(fig,FIGURES[8],L('同一几何与任务：误差和销柔度的设计取舍','Same geometry and duty: trade-offs in errors and pin compliance'))


def main():
    parser=argparse.ArgumentParser(description='复算DFKI摆台参照再设计案例并生成中英双版图表')
    parser.add_argument('--output',type=Path,default=project_dir()/'examples/results/dfki_pendulum')
    parser.add_argument('--raw-output',type=Path,help='可选的完整原接触数组输出目录')
    args=parser.parse_args();root=project_dir()
    base=json.loads((root/'examples/dfki_pendulum_case.json').read_text(encoding='utf-8'))
    requirements=json.loads((root/'examples/dfki_pendulum_requirements.json').read_text(encoding='utf-8'))
    generated,_=pendulum_duty(requirements)
    validate_saved_duty(generated,base['duty_segments'])
    results=[analyze_joint(c) for c in candidate_configs(base)]
    data=prepare_data(base,requirements,results)
    args.output.mkdir(parents=True,exist_ok=True)
    write_json(args.output/'metrics.json',data['metrics'])
    write_json(args.output/'summary.json',results[0].summary)
    write_json(args.output/'design_comparison.json',data['metrics']['candidate_comparison'])
    for language in ('zh','en'):render_figures(data,args.output,language)
    with (args.output/'duty.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['time_s','theta_rad','output_torque_n_m','output_speed_rad_s','load_share_factor','output_stiffness_n_m_per_rad'])
        for state,segment,case in zip(data['derived']['states'],base['duty_segments'],results[0].cases):
            writer.writerow([state['time_s'],state['theta_rad'],segment['output_torque_n_m'],segment['output_speed_rad_s'],case['sharing']['load_share_factor'],case['precision']['output_torsional_stiffness_n_m_per_rad']])
    from robot_joint.engineering import export_engineering_parameters
    export_engineering_parameters(results[0],args.output)
    if args.raw_output:export_joint(results[0],args.raw_output)
    print(json.dumps({'figures_per_language':len(FIGURES),'states':len(results[0].cases),
      'peak_discretization_difference':data['metrics']['torque_peak_discretization_relative_difference']},ensure_ascii=False))


if __name__=='__main__':main()
