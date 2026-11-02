"""从已导出销/行星轴承和DC母线结果生成同源中英蓝红论文图。"""
from pathlib import Path
import argparse
import csv
import json
import numpy as np
from robot_joint.figure_style import COLORS,publication_settings,save_publication_figure


def plot_screens(pin_folder,bus_folder,output,languages=('zh','en')):
    """显示静强度/刚度一致性及母线能量约束，保持新设计假设与原产品规格区分。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    read=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
    pins=read(Path(pin_folder)/'summary.json');bus=read(Path(bus_folder)/'study.json')
    with (Path(pin_folder)/'history.csv').open(encoding='utf-8-sig',newline='') as f:history=list(csv.DictReader(f))
    candidates=pins['candidates']
    for language in languages:
        L=lambda zh,en:zh if language=='zh' else en
        with plt.rc_context(publication_settings(language)):
            fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
            first=[row for row in history if row['candidate_id']==candidates[0]['id']]
            for planet in sorted({int(row['planet_index']) for row in first}):
                rows=[row for row in first if int(row['planet_index'])==planet]
                durations=np.array([float(row['duration_s']) for row in rows]);time=np.cumsum(durations)-durations/2
                axes[0,0].plot(time,[float(row['pin_load_n']) for row in rows],['o-','s--','^-.'][planet%3],
                              color=COLORS[planet%3],label=L(f'行星{planet+1}',f'Planet {planet+1}'))
            axes[0,0].set(xlabel=L('任务时间 / s','Duty time / s'),ylabel=L('销横向载荷 / N','Transverse pin load / N'),title=L('由系统反力获得真实载荷需求','Load demand from solved system reactions'));axes[0,0].legend()
            x=np.arange(len(candidates[0]['pins']));labels=[]
            for index,candidate in enumerate(candidates):
                diameter=pins['assumptions']['candidates'][index]['pins']['diameter_m']*1000
                label=L(f'{diameter:g} mm销',f'{diameter:g} mm pin');labels.append(label)
                axes[0,1].bar(x+(index-.5)*.34,[row['equivalent_pin_stiffness_n_per_m']/1e6 for row in candidate['pins']],
                              width=.34,color=COLORS[index],label=label)
            given=[row['specified_pin_stiffness_n_per_m']/1e6 for row in candidates[0]['pins']]
            axes[0,1].plot(x,given,'k--',label=L('原系统输入','Original system input'),lw=1)
            axes[0,1].set(xticks=x,xticklabels=[str(j+1) for j in x],xlabel=L('行星编号','Planet'),ylabel='MN/m',title=L('载荷共轭弯剪刚度；不自动回写分载','Load-conjugate bending/shear stiffness; no feedback'));axes[0,1].legend(fontsize=8)
            field_labels=[L('弯曲','Bending'),L('横剪切','Shear'),L('平均孔压','Projected pressure')]
            for index,candidate in enumerate(candidates):
                values=[max(row[key] for row in candidate['pins'])/1e6 for key in
                        ('maximum_bending_stress_pa','maximum_transverse_shear_stress_pa','maximum_projected_average_pressure_pa')]
                axes[1,0].bar(np.arange(3)+(index-.5)*.34,values,width=.34,color=COLORS[index],label=labels[index])
            axes[1,0].set(xticks=np.arange(3),xticklabels=field_labels,ylabel='MPa',title=L('不同位置的应力分别评价','Separate stress measures at different locations'));axes[1,0].legend()
            axes[1,1].bar(labels,[max(row['maximum_center_deflection_m'] for row in candidate['pins'])*1e6 for candidate in candidates],color=COLORS[:len(candidates)])
            axes[1,1].set(ylabel=L('中心挠度 / μm','Centre deflection / μm'),title=L('短粗销仍须三维支承/孔接触校核','Short pins still need 3D support/hole-contact checks'))
            for ax in axes.flat:ax.grid(axis='y',alpha=.14)
            save_publication_figure(fig,Path(output)/language/'01_planet_support',language=language,
                title=L('继续设计：行星销弯剪、静强度与刚度一致性','Further design: planet-pin bending/shear, static strength and stiffness consistency'),
                footnote=L('销几何/材料限值/轴承容量均为新增假设；基本L10不是实物摆动寿命。','Pin geometry, allowables and bearing capacities are new assumptions; basic L10 is not oscillatory hardware life.'))
            plt.close(fig)
            fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
            names=[L('470 μF，无吸收','470 μF, no sink'),L('4700 μF，无吸收','4700 μF, no sink'),L('470 μF，15 W吸收','470 μF, 15 W sink')]
            case=bus['cases'][0];segments=case['segments']
            times=[row['begin_time_s'] for row in segments]+[segments[-1]['end_time_s']]
            for field,label,color in [('motor_electrical_power_w',L('电机侧电功率','Motor electrical power'),COLORS[0]),('dc_power_w',L('DC侧功率','DC power'),COLORS[1])]:
                values=[row[field] for row in segments]
                axes[0,0].step(times,values+[values[-1]],where='post',color=color,label=label)
            axes[0,0].axhline(0,color='#777777',lw=.6)
            axes[0,0].set(xlabel=L('任务时间 / s','Duty time / s'),ylabel='W',title=L('扣除电机损耗后才得到真实回馈需求','Regeneration is assessed after motor losses'));axes[0,0].legend()
            for j,case in enumerate(bus['cases']):
                t=[];v=[];p=case['parameters']
                for row in case['segments']:
                    local=np.linspace(0,row['duration_s'],31)
                    energy=np.maximum(0,row['begin_excess_capacitor_energy_j']+row['net_capacitor_power_before_floor_w']*local)
                    t.extend(row['begin_time_s']+local);v.extend(np.sqrt(p['source_voltage_v']**2+2*energy/p['capacitance_f']))
                axes[0,1].plot(t,v,['-','--','-.'][j],color=COLORS[j],label=names[j])
            axes[0,1].axhline(case['parameters']['maximum_voltage_v'],color=COLORS[1],ls=':',lw=1)
            axes[0,1].set(xlabel=L('任务时间 / s','Duty time / s'),ylabel='V',title=L('母线自由能量轨迹与48 V上限','Free-energy bus trajectory and the 48 V limit'));axes[0,1].legend(fontsize=8)
            x=np.arange(len(names))
            axes[1,0].bar(x-.17,[row['maximum_excess_capacitor_energy_j'] for row in bus['cases']],width=.34,color=COLORS[0],label=L('峰值储能需求','Peak buffer demand'))
            axes[1,0].bar(x+.17,[row['capacitor_energy_capacity_to_maximum_voltage_j'] for row in bus['cases']],width=.34,color=COLORS[1],label=L('至上限的电容容量','Capacity to voltage limit'))
            axes[1,0].set(xticks=x,xticklabels=[L('小电容','Small C'),L('大电容','Large C'),L('带吸收','With sink')],ylabel='J',title=L('能量需求与容许容量分别比较','Compare energy demand with available capacity'));axes[1,0].legend(fontsize=8)
            axes[1,1].bar(x,[row['minimum_capacitance_f_for_nominal_start']*1e6 for row in bus['cases']],color=COLORS[:3])
            axes[1,1].set(xticks=x,xticklabels=[L('小电容','Small C'),L('大电容','Large C'),L('带吸收','With sink')],ylabel='μF',title=L('源电压起始的数学最低电容','Mathematical minimum C from nominal start'))
            for ax in axes.flat:ax.grid(axis='y',alpha=.14)
            save_publication_figure(fig,Path(output)/language/'02_dc_bus',language=language,
                title=L('继续设计：回馈能量、母线过压与吸收能力','Further design: regeneration, DC-bus overvoltage and absorption capacity'),
                footnote=L('电源/效率/C为新假设；过压后的自由轨迹不模拟真实保护，未验证PWM/电流/电阻。','Source, efficiency and C are assumptions; beyond-limit trajectories do not model actual protection, PWM, current or braking resistors.'))
            plt.close(fig)


def main():
    root=next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    parser=argparse.ArgumentParser(description='重绘销与母线设计初筛的中英图表')
    parser.add_argument('--pin-results',type=Path,default=root/'examples/results/planet_support')
    parser.add_argument('--bus-results',type=Path,default=root/'examples/results/dc_bus')
    parser.add_argument('--output',type=Path,default=root/'examples/results/design_screens')
    parser.add_argument('--language',choices=['zh','en','both'],default='both')
    args=parser.parse_args();plot_screens(args.pin_results,args.bus_results,args.output,('zh','en') if args.language=='both' else (args.language,))


if __name__=='__main__':main()
