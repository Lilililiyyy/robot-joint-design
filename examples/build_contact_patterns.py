"""复算正反峰载与低载的完整接触斑点，并导出双语论文图及原采样数组。"""
from copy import deepcopy
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
from gear_contact import analyze_config
from gear_contact.io import write_json,export_result
from robot_joint.contact_pattern import build_contact_pattern,export_contact_pattern,interior_snapshot
from robot_joint.figure_style import COLORS,publication_settings,save_publication_figure


def project_root():
    return next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())


def bundle_from_export(folder):
    """读取调用者指定export_joint结果，按SHA绑定局部输入及系统载荷。"""
    folder=Path(folder);read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
    manifest=read(folder/'manifest.json')
    def verified(name):
        path=folder/name
        if name not in manifest['files'] or hashlib.sha256(path.read_bytes()).hexdigest()!=manifest['files'][name]:
            raise ValueError('已导出结果身份不匹配：'+name)
        return read(path)
    base=verified('input.json');segments=base['duty_segments']
    if not any(s['output_torque_n_m']>0 for s in segments) or not any(s['output_torque_n_m']<0 for s in segments):
        raise ValueError('此正反案例命令需要两种扭矩方向；单副结果可直接调用build_contact_pattern')
    selected={'positive_peak':max(range(len(segments)),key=lambda j:segments[j]['output_torque_n_m']),
              'negative_peak':min(range(len(segments)),key=lambda j:segments[j]['output_torque_n_m']),
              'low_load':min(range(len(segments)),key=lambda j:abs(segments[j]['output_torque_n_m']))}
    cases=[]
    for name,j in selected.items():
        prefix=f'case_{j+1:03d}';sharing=verified(prefix+'/sharing.json')
        case={'name':name,'system_case_index':j,'duty':segments[j],
              'system_load_share_factor':sharing['load_share_factor'],
              'system_path_residual_m':sharing['coupling']['path_residual_m'],'meshes':[]}
        for mesh in ('sun_planet','planet_ring'):
            for i in range(base['stage']['planet_count']):
                relative=f'{prefix}/{mesh}/planet_{i+1:02d}/input.json'
                case['meshes'].append({'mesh':mesh,'planet':i+1,
                    'frozen_tangential_force_n':sharing['tangential_forces_n'][i],
                    'source_input_sha256':manifest['files'][relative],'analysis':verified(relative)})
        cases.append(case)
    return {'schema_version':1,'source_joint_case_id':base['case_id'],
            'source_joint_input_sha256':manifest['files']['input.json'],
            'system_resolution':[base['mesh']['phases'],base['mesh']['strips']],
            'cases':cases,'scope_zh':'在系统已协调的局部载荷和边界下加密单副；不反向更新原系统刚度/分载。'}


def calculate(bundle, output, *, phases=512, strips=41, profile_points=1601, raw_output=None):
    """每副重新求解加密相位，不读取任何参考响应作为压力输入。"""
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    write_json(output/'inputs.json',bundle)
    rows=[];patterns={}
    for case in bundle['cases']:
        for item in case['meshes']:
            config=deepcopy(item['analysis']);config.update(phases=phases,strips=strips)
            result=analyze_config(config)
            snapshot,tooth=interior_snapshot(result)
            key=(case['name'],item['mesh'],item['planet'])
            patterns[key]={}
            name=f"{case['name']}/{item['mesh']}/planet_{item['planet']:02d}"
            if raw_output:export_result(result,Path(raw_output)/name)
            for member in ('pinion','wheel'):
                pattern=build_contact_pattern(result,member=member,profile_points=profile_points,
                    snapshot_index=snapshot,snapshot_tooth_pair_index=tooth)
                pattern.summary['frozen_tangential_force_n']=item['frozen_tangential_force_n']
                pattern.summary['signed_output_torque_n_m']=case['duty']['output_torque_n_m']
                pattern.summary['loaded_flank']='positive' if case['duty']['output_torque_n_m']>=0 else 'negative'
                export_contact_pattern(pattern,output/'data'/name/member)
                patterns[key][member]=pattern
                rows.append({'case':case['name'],'mesh':item['mesh'],'planet':item['planet'],
                    'frozen_tangential_force_n':item['frozen_tangential_force_n'],
                    'source_input_sha256':item['source_input_sha256'],
                    'source_analysis_summary':result.summary,**pattern.summary})
            print(json.dumps({'已求解接触副':name,'phases':phases,'strips':strips},ensure_ascii=False),flush=True)
    summary={'source_joint_case_id':bundle['source_joint_case_id'],'system_resolution':bundle['system_resolution'],
        'phases':phases,'strips':strips,'profile_points':profile_points,'solved_meshes':len(rows)//2,
        'surface_patterns':len(rows),'patterns':rows,'interpolated_pressure':False,
        'system_load_sharing_was_recomputed':False,'physical_validation':False,
        'scope':'新写有限Hertz带后处理，周期相同齿等价拼接；非二维/FE接触、非实测斑点；局部加密未反馈原系统协调解'}
    write_json(output/'summary.json',summary)
    return patterns,summary


def load_patterns(output):
    """仅重绘已保存图表数据，不新增求解相位或插值填色。"""
    from robot_joint.contact_pattern import ContactPattern
    output=Path(output);summary=json.loads((output/'summary.json').read_text(encoding='utf-8'))
    patterns={}
    for row in summary['patterns']:
        key=(row['case'],row['mesh'],row['planet']);folder=output/'data'/row['case']/row['mesh']/f"planet_{row['planet']:02d}"/row['member']
        manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
        for name,expected in manifest['files'].items():
            if hashlib.sha256((folder/name).read_bytes()).hexdigest()!=expected:
                raise ValueError('接触图数据身份不匹配：'+str(folder/name))
        with np.load(folder/'pattern.npz',allow_pickle=False) as arrays:
            pattern=ContactPattern({name:arrays[name].copy() for name in arrays.files},json.loads((folder/'summary.json').read_text(encoding='utf-8')))
        patterns.setdefault(key,{})[row['member']]=pattern
    return patterns,summary


def plot_patterns(patterns, summary, output, languages=('zh','en')):
    """瞬时单齿带、完整扫掠及逐行星图共用实际MPa色标，空白不平滑补齐。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Patch
    cmap=LinearSegmentedColormap.from_list('pressure_blue_red',['#2166ac','#9dc7e0','#f2d1d0','#b2182b'])
    cmap.set_bad((0,0,0,0))
    maximum=max(row['source_peak_pressure_pa'] for row in summary['patterns'])/1e6
    output=Path(output)
    best={}
    for mesh in ('sun_planet','planet_ring'):
        choices=[(key,value) for key,value in patterns.items() if key[0]=='negative_peak' and key[1]==mesh]
        best[mesh]=max(choices,key=lambda item:item[1]['pinion'].summary['source_peak_pressure_pa'])
    for language in languages:
        L=lambda zh,en:zh if language=='zh' else en
        def draw(ax,pattern,instant=False):
            arrays=pattern.arrays;s=pattern.summary
            x=arrays['face_edges_m']*1000
            y=arrays['snapshot_profile_roll_edges_m' if instant else 'profile_roll_edges_m']*1000
            ax.set_facecolor('#e8ebee')
            low,high=s['theoretical_contact_roll_bounds_m']
            ax.axhspan(low*1000,high*1000,color='white',zorder=0)
            pressure=arrays['instant_pressure_pa' if instant else 'sweep_pressure_pa']/1e6
            artist=ax.pcolormesh(x,y,np.ma.masked_less_equal(pressure,0),cmap=cmap,vmin=0,vmax=maximum,
                shading='flat',rasterized=True)
            unknown=arrays['unresolved_sampling_mask']&arrays['theoretical_path_mask'][:,None]
            if not instant and np.any(unknown):
                # 网纹仅为相邻正载样本间的未解析空隙，不代表所有可能未知域。
                ax.contourf((x[:-1]+x[1:])/2,(y[:-1]+y[1:])/2,unknown.astype(float),levels=[.5,1.5],colors=['none'],hatches=['////'])
            full_low,full_high=s['full_effective_flank_roll_bounds_m']
            ax.set(xlim=(x[0],x[-1]),ylim=(full_low*1000,full_high*1000),xlabel=L('齿宽位置 / mm','Face position / mm'),
                   ylabel=L('本轮渐开线展长 / mm','Member involute roll / mm'))
            return artist
        def save(fig,name,title):
            save_publication_figure(fig,output/language/name,title=title,language=language,
                footnote=L('局部Hertz带后处理；灰区为理论路径外，白区为未记录正压；空隙及边缘误差见数据。',
                  'Local Hertz-band postprocessing; grey: outside path; white: no sampled pressure. Gaps/edge errors are reported.'))
            plt.close(fig)
        with plt.rc_context(publication_settings(language)):
            fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
            for col,mesh in enumerate(('sun_planet','planet_ring')):
                key,values=best[mesh];pattern=values['pinion']
                label=L('太阳轮—行星轮','Sun–planet') if col==0 else L('行星轮—内齿圈','Planet–ring')
                for row in range(2):
                    artist=draw(axes[row,col],pattern,instant=row==0)
                    axes[row,col].set_title(label+'\n'+L('内域相位的一对齿瞬时接触带' if row==0 else '周期等价完整单齿通过程最大压力',
                        'One tooth pair at an interior phase' if row==0 else 'Full tooth-passage sampled pressure envelope'))
                    fig.colorbar(artist,ax=axes[row,col],label='MPa',shrink=.8)
            save(fig,'01_contact_overview',L('完整接触斑点：瞬时单齿带与整个齿通过程','Complete contact patterns: one-pair snapshot and full tooth passage'))
            for case,name in [('positive_peak','02_positive_planets'),('negative_peak','03_negative_planets'),('low_load','04_low_load_planets')]:
                n=max(key[2] for key in patterns if key[0]==case)
                fig,axes=plt.subplots(n,2,figsize=(12,11),layout='constrained',squeeze=False)
                for j in range(n):
                    for col,mesh in enumerate(('sun_planet','planet_ring')):
                        pattern=patterns[(case,mesh,j+1)]['pinion'];artist=draw(axes[j,col],pattern)
                        label=L('太阳轮副','S–P') if col==0 else L('内齿副','P–R')
                        axes[j,col].set_title(label+L(f'，行星{j+1}，Ft=',f', planet {j+1}, Ft=')+
                            f"{pattern.summary['frozen_tangential_force_n']:.2f} N")
                        fig.colorbar(artist,ax=axes[j,col],label='MPa',shrink=.75)
                title={'positive_peak':L('正向峰载','Positive peak load'),'negative_peak':L('反向峰载','Negative peak load'),'low_load':L('低载与未承载边缘','Low load and unloaded face regions')}[case]
                save(fig,name,title+L('：三行星、两副的完整通过程斑点',': full-passage patterns of three planets and both meshes'))
            fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
            for row,mesh in enumerate(('sun_planet','planet_ring')):
                key,values=best[mesh]
                for col,member in enumerate(('pinion','wheel')):
                    artist=draw(axes[row,col],values[member])
                    label={('sun_planet','pinion'):L('太阳轮','Sun'),('sun_planet','wheel'):L('行星轮：太阳副侧','Planet: sun mesh'),
                           ('planet_ring','pinion'):L('行星轮：内齿副侧','Planet: ring mesh'),('planet_ring','wheel'):L('内齿圈','Ring')}[(mesh,member)]
                    axes[row,col].set_title(label)
                    fig.colorbar(artist,ax=axes[row,col],label='MPa',shrink=.8)
            save(fig,'05_members',L('两副四个齿面：使用各轮自己的材料展长','Four member flanks: each uses its own material roll coordinate'))


def main():
    root=project_root();parser=argparse.ArgumentParser(description='生成完整齿通过程Hertz接触斑点与中英图表')
    parser.add_argument('--input',type=Path,default=root/'examples/contact_pattern_inputs.json')
    parser.add_argument('--joint-results',type=Path,help='可选的自有export_joint目录，覆盖默认局部输入集合')
    parser.add_argument('--output',type=Path,default=root/'examples/results/contact_patterns')
    parser.add_argument('--raw-output',type=Path,help='可选，另存全部LTCA原数组/诊断/CSV')
    parser.add_argument('--phases',type=int,default=512);parser.add_argument('--strips',type=int,default=41)
    parser.add_argument('--profile-points',type=int,default=1601)
    parser.add_argument('--language',choices=['zh','en','both'],default='both')
    parser.add_argument('--replot',action='store_true',help='只从既有NPZ重绘，不重新求解')
    args=parser.parse_args()
    if args.replot:patterns,summary=load_patterns(args.output)
    else:
        if args.joint_results:
            source=args.joint_results.resolve();target=args.output.resolve()
            if source==target or source in target.parents or target in source.parents:
                parser.error('接触输出与原关节结果必须位于互不包含的目录，避免覆盖原证据')
        if args.input.resolve()==(args.output/'inputs.json').resolve():
            parser.error('输出inputs.json不能覆盖作为计算输入的原文件')
        bundle=bundle_from_export(args.joint_results) if args.joint_results else json.loads(args.input.read_text(encoding='utf-8'))
        patterns,summary=calculate(bundle,args.output,phases=args.phases,strips=args.strips,
            profile_points=args.profile_points,raw_output=args.raw_output)
    plot_patterns(patterns,summary,args.output,('zh','en') if args.language=='both' else (args.language,))
    print(json.dumps({'solved_meshes':summary['solved_meshes'],'surface_patterns':summary['surface_patterns'],'figures_per_language':5},ensure_ascii=False))


if __name__=='__main__':main()
