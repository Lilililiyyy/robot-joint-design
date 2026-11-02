"""MASTA单副接触压力结果对标，独立求解后在相同材料坐标网格比较。"""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np

from gear_contact import analyze_config
from gear_contact.io import write_json
from robot_joint.contact_display import contact_pass_data, draw_contact_pass
from robot_joint.figure_style import publication_settings, save_publication_figure


def calculate(output):
    """参照压力只参与差值计算，不进入求解；保存正式对标数据供重绘。"""
    output=Path(output).resolve()
    root=next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    reference=(root/'benchmarks/reference/masta_18_77').resolve()
    if output==reference or reference in output.parents or output in reference.parents:
        raise ValueError('对标结果不能覆盖参照目录及其父子路径')
    metadata=json.loads((reference/'provenance.json').read_text(encoding='utf-8'))
    for name in ('display.npz','display_input.json'):
        if hashlib.sha256((reference/name).read_bytes()).hexdigest()!=metadata['files'][name]:
            raise ValueError('对标输入或参考数据身份不一致：'+name)
    config=json.loads((reference/'display_input.json').read_text(encoding='utf-8'))
    result=analyze_config(config)
    with np.load(reference/'display.npz',allow_pickle=False) as source:
        x=source['face_coordinate_m'];y=source['roll_distance_m'];pressure=source['pressure_pa_x_y']
        data=contact_pass_data(result,face_coordinate_m=x,roll_distance_m=y)
    project=data['pressure_pa_x_y'];valid=np.isfinite(project)&np.isfinite(pressure)
    difference=project[valid]-pressure[valid]
    metrics=dict(reference_name='MASTA',case_id='helical_18_77',grid_shape_x_y=list(pressure.shape),
        common_finite_count=int(np.count_nonzero(valid)),project_uncovered_count=int(np.count_nonzero(~np.isfinite(project))),
        mae_pa=float(np.mean(np.abs(difference))),rmse_pa=float(np.sqrt(np.mean(difference**2))),
        maximum_absolute_difference_pa=float(np.max(np.abs(difference))),
        reference_display_peak_pa=float(np.nanmax(pressure)),project_display_peak_pa=float(np.nanmax(project)),
        project_raw_endpoint_peak_pa=float(data['samples'][:,2].max()),
        whole_joint_benchmark=False,physical_validation=False,
        scope_zh='同一单副输入的通过程压力结果对标，使用已保存参照；端点压力与显示压力分别定义。')
    output.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(output/'contact_pressure.npz',face_coordinate_m=x,roll_distance_m=y,
        reference_pressure_pa_x_y=pressure,project_pressure_pa_x_y=project)
    write_json(output/'contact_pressure.json',metrics)
    write_json(output/'contact_pressure_manifest.json',dict(files={name:hashlib.sha256((output/name).read_bytes()).hexdigest()
        for name in ('contact_pressure.npz','contact_pressure.json')}))
    return metrics


def plot(output,languages=('zh','en')):
    """读取同一对标结果绘制双语蓝红压力图。"""
    output=Path(output)
    manifest=json.loads((output/'contact_pressure_manifest.json').read_text(encoding='utf-8'))
    for name,expected in manifest['files'].items():
        if hashlib.sha256((output/name).read_bytes()).hexdigest()!=expected:
            raise ValueError('对标绘图数据身份不一致：'+name)
    with np.load(output/'contact_pressure.npz',allow_pickle=False) as source:
        arrays={key:source[key].copy() for key in source.files}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    maximum=max(np.nanmax(arrays['reference_pressure_pa_x_y']),np.nanmax(arrays['project_pressure_pa_x_y']))
    for language in languages:
        L=lambda zh,en:zh if language=='zh' else en
        with plt.rc_context(publication_settings(language)):
            fig,axes=plt.subplots(1,2,figsize=(12,6.3),layout='constrained')
            for index,field in enumerate(('reference_pressure_pa_x_y','project_pressure_pa_x_y')):
                data=dict(face_coordinate_m=arrays['face_coordinate_m'],roll_distance_m=arrays['roll_distance_m'],
                    pressure_pa_x_y=arrays[field],interpolated=True)
                artist=draw_contact_pass(axes[index],data,maximum_pa=maximum,language=language,
                    title='MASTA' if index==0 else L('本项目','This project'))
                fig.colorbar(artist,ax=axes[index],label='MPa',shrink=.8)
            save_publication_figure(fig,output/language/'02_contact_pressure',language=language,
                title=L('18/77斜齿轮副：通过程接触压力对标','18/77 helical mesh: tooth-passage contact pressure benchmark'),
                footnote=L('相同输入与30×30材料坐标网格；灰色为支撑域外；对标范围为指定单副工况。',
                    'Matching inputs and 30×30 material grid; grey: outside support; comparison covers the stated single mesh.'))
            plt.close(fig)


def main():
    root=next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=root/'benchmarks/results')
    parser.add_argument('--language',choices=('zh','en','both'),default='both')
    parser.add_argument('--replot',action='store_true')
    args=parser.parse_args()
    if not args.replot:calculate(args.output)
    plot(args.output,('zh','en') if args.language=='both' else (args.language,))


if __name__=='__main__':main()
