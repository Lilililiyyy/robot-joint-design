"""由完整关节输入复算销尺寸、系统分载及最终零件校核的比较。"""
from copy import deepcopy
from pathlib import Path
import argparse
import csv
import hashlib
import json
import math
import tempfile
import numpy as np

from robot_joint import analyze_joint, export_joint, resolve_pin_stiffness
from gear_contact.io import write_json


def project_root():
    return next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())


def _responses(result):
    """手工回填与几何入口比较原响应，不比较显示插值或文本来源标签。"""
    arrays={}
    for i,case in enumerate(result.cases):
        for field in ('tangential_forces_n','carrier_pin_forces_xy_n','compliance_m_per_n'):
            arrays[f'case_{i:03d}_{field}']=np.asarray(case['sharing'][field])
        for mesh,contacts in case['contacts'].items():
            for j,contact in enumerate(contacts):
                stem=f'case_{i:03d}_{mesh}_{j:02d}_'
                arrays[stem+'te_m']=np.asarray(contact.normal_approach_m)
                arrays[stem+'stiffness_n_per_m']=np.asarray(contact.mesh_stiffness_n_per_m)
                arrays[stem+'pressure_pa']=np.concatenate([r.pressure_pa for r in contact.rotations])
    return arrays


def run(config, output, *, smaller_diameter_m=.004, raw_output=None):
    """全部方案成功后再替换正式比较文件，失败时保留上一次完整结果。"""
    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.pin_comparison_', dir=destination.parent) as temporary:
        stage = Path(temporary)
        summary = _calculate_comparison(config, stage,
            smaller_diameter_m=smaller_diameter_m, raw_output=raw_output)
        destination.mkdir(parents=True, exist_ok=True)
        # 完成清单最后替换，所有正式文件均在真实计算完成后发布。
        names = [p for p in stage.iterdir() if p.name != 'manifest.json']+[stage/'manifest.json']
        for path in names:
            path.replace(destination/path.name)
    return summary


def _calculate_comparison(config, output, *, smaller_diameter_m, raw_output):
    """实际重算四个完整任务，原请求和全部派生输入分别保存。

    基准销参数来自调用者。较小销径为显式比较参数，示例4/6mm不是产品
    规格或采购选型；各方案保持其余输入和轴承容量相同以观察影响。
    """
    base=deepcopy(config)
    count=base['stage']['planet_count']
    support=base['planet_support']
    pins=support['pins']
    per_pin=([deepcopy(pins) for _ in range(count)] if isinstance(pins,dict) else deepcopy(pins))
    if (isinstance(smaller_diameter_m,bool) or not math.isfinite(smaller_diameter_m)
            or not 0 < smaller_diameter_m < min(p['diameter_m'] for p in per_pin)):
        raise ValueError('比较销径须为正有限m值，且小于每个基准销直径')
    adopted=resolve_pin_stiffness(count,{'pin_stiffness_source':'pin_geometry'},pins=pins)
    variants={}
    for name in ('manual_reference','geometry_reference','all_smaller','first_smaller'):
        candidate=deepcopy(base)
        candidate['case_id']=base['case_id']+'_'+name
        candidate['load_sharing'].pop('pin_stiffness_n_per_m',None)
        candidate['load_sharing']['pin_stiffness_source']='pin_geometry'
        if name=='manual_reference':
            candidate['load_sharing'].pop('pin_stiffness_source')
            candidate['load_sharing']['pin_stiffness_n_per_m']=adopted['adopted_pin_stiffness_n_per_m']
        elif name in ('all_smaller','first_smaller'):
            candidate['planet_support']['pins']=deepcopy(per_pin)
            for j in (range(count) if name=='all_smaller' else (0,)):
                candidate['planet_support']['pins'][j]['diameter_m']=smaller_diameter_m
        variants[name]=candidate
    output=Path(output)
    output.mkdir(parents=True,exist_ok=True)
    write_json(output/'source_input.json',base)
    rows=[];records=[];saved={};manual=None;equivalence=None
    for name,candidate in variants.items():
        write_json(output/(name+'_input.json'),candidate)
        result=analyze_joint(candidate)
        if name=='manual_reference':
            manual=_responses(result)
        elif name=='geometry_reference':
            current=_responses(result)
            errors={key:float(np.max(np.abs(current[key]-old))) for key,old in manual.items()}
            matched=all(np.array_equal(current[key],old) for key,old in manual.items())
            equivalence={'all_response_arrays_identical':matched,'compared_arrays':len(errors),
                         'maximum_absolute_differences_by_array':errors}
            if not matched:
                raise RuntimeError('几何来源与等值手工回填的原响应不一致，停止交付比较')
            manual=None
        if raw_output is not None:
            export_joint(result,Path(raw_output)/name)
        saved[name+'_forces_n']=np.asarray([c['sharing']['tangential_forces_n'] for c in result.cases])
        saved[name+'_adopted_pin_stiffness_n_per_m']=np.asarray(result.pin_stiffness['adopted_pin_stiffness_n_per_m'])
        record={'scenario':name,'case_id':candidate['case_id'],'duty_states':len(result.cases),
            'pin_diameters_m':[p['diameter_m'] for p in
                ([candidate['planet_support']['pins']]*count if isinstance(candidate['planet_support']['pins'],dict)
                 else candidate['planet_support']['pins'])],
            'adopted_pin_stiffness_n_per_m':result.pin_stiffness['adopted_pin_stiffness_n_per_m'],
            'maximum_load_share_factor':result.summary['maximum_load_share_factor'],
            'maximum_contact_pressure_pa':result.summary['maximum_contact_pressure_pa'],
            'minimum_output_stiffness_n_m_per_rad':result.summary['minimum_output_torsional_stiffness_n_m_per_rad'],
            'maximum_pin_load_n':result.planet_support['summary']['maximum_pin_load_n'],
            'maximum_pin_bending_stress_pa':max(p['maximum_bending_stress_pa'] for p in result.planet_support['pins']),
            'minimum_planet_bearing_static_safety_factor':result.summary['minimum_planet_bearing_static_safety_factor'],
            'maximum_torque_residual_n_m':max(abs(c['sharing']['diagnostics']['output_torque_balance_n_m']) for c in result.cases),
            'maximum_path_residual_m':max(c['sharing']['coupling']['path_residual_m'] for c in result.cases),
            'all_couplings_converged':result.summary['all_couplings_converged'],
            'configured_checks_pass':result.summary['configured_checks_pass'],
            'failed_configured_checks':[key for key,item in result.summary['check_details'].items()
                if item['status']=='failed'],
            'short_pin_review_required':any(p['short_pin_review_required'] for p in result.planet_support['pins']),
            'pin_stiffness':result.pin_stiffness,'physical_validation':False}
        records.append(record)
        for i,case in enumerate(result.cases):
            for j,force in enumerate(case['sharing']['tangential_forces_n']):
                pin=result.planet_support['cases'][i]['pins'][j]
                rows.append({'scenario':name,'case_index':i,'label':case['label'],'planet_index':j,
                    'output_torque_n_m':candidate['duty_segments'][i]['output_torque_n_m'],
                    'adopted_pin_stiffness_n_per_m':result.pin_stiffness['adopted_pin_stiffness_n_per_m'][j],
                    'tangential_force_n':float(force),'pin_load_n':pin['load_n'],
                    'pin_bending_stress_pa':pin['bending_stress_pa'],
                    'maximum_contact_pressure_pa':max(case['contacts'][mesh][j].summary['max_pressure_pa']
                        for mesh in ('sun_planet','planet_ring')),
                    'path_residual_m':case['sharing']['coupling']['path_residual_m']})
        print(json.dumps({'已完成销刚度方案':name,'工况数':len(result.cases)},ensure_ascii=False),flush=True)
    summary={'records':records,'manual_equivalence':equivalence,'physical_validation':False,
        'scope':'销参数与既有降阶分载/LTCA的数值接入比较；保持示例轴承容量，不代表三维支承或实物寿命验证'}
    write_json(output/'summary.json',summary)
    np.savez_compressed(output/'comparison.npz',**saved)
    with (output/'sharing.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    labels={'manual_reference':'等值手工回填','geometry_reference':'几何来源基准',
            'all_smaller':'全部销减小','first_smaller':'仅第一销减小'}
    report=['# 销参数、系统分载与校核比较','',
        '完整输入和各派生输入随结果保存；所有方案重新协调系统及两副齿面，再使用最终销反力校核。','',
        '逐销直径 / mm：'+ '；'.join(labels[r['scenario']]+': '+
            ' / '.join(f'{v*1e3:g}' for v in r['pin_diameters_m']) for r in records)+'。','',
        '|方案|采用刚度 / MN/m|最大均载系数|峰压 / MPa|最小输出刚度 / N·m/rad|最大销弯曲 / MPa|最小行星轴承静安全|配置约束|',
        '|---|---|---:|---:|---:|---:|---:|---|']
    for row in records:
        k=' / '.join(f'{v/1e6:.3f}' for v in row['adopted_pin_stiffness_n_per_m'])
        report.append(f"|{labels[row['scenario']]}|{k}|{row['maximum_load_share_factor']:.6f}|"
            f"{row['maximum_contact_pressure_pa']/1e6:.3f}|{row['minimum_output_stiffness_n_m_per_rad']:.3f}|"
            f"{row['maximum_pin_bending_stress_pa']/1e6:.3f}|{row['minimum_planet_bearing_static_safety_factor']:.6f}|"
            f"{'通过' if row['configured_checks_pass'] else '未通过'}|")
    report+=['',f"等值手工回填与几何来源的{equivalence['compared_arrays']}份原响应数组逐值相同。",'',
        f"{len(records)}个方案、每个{len(rows)//(len(records)*count)}个任务状态："
        f"{'协调全部收敛' if all(r['all_couplings_converged'] for r in records) else '存在协调未收敛状态'}；"
        f"最大输出扭矩残差{max(r['maximum_torque_residual_n_m'] for r in records):.3g} N·m，"
        f"最大路径残差{max(r['maximum_path_residual_m'] for r in records):.3g} m。",'',
        '未通过的已配置约束：'+('；'.join(labels[r['scenario']]+': '+', '.join(r['failed_configured_checks'])
            for r in records if r['failed_configured_checks']) or '无')+'。','',
        '短粗销示例仍采用简支梁与横剪柔度，尚未评价销孔压陷、架板变形、配合和三维接触。轴承容量在本比较中保持给定值；实际变径需重新选型。',
        '来源一致性、扭矩平衡和数值收敛属于模型接入验证，不是关节总成物理验证。','']
    (output/'report.md').write_text('\n'.join(report),encoding='utf-8',newline='\n')
    english_labels={'manual_reference':'Manual equivalent input','geometry_reference':'Geometry source',
                    'all_smaller':'All pins smaller','first_smaller':'First pin smaller'}
    english=['# Pin geometry, sharing and final checks','',
        'All four complete designs are recalculated through system/mesh coordination; final pin reactions drive pin and bearing checks. Inputs, actual stiffnesses and hashes are retained.','',
        'Pin diameters / mm: '+'; '.join(english_labels[r['scenario']]+': '+
            ' / '.join(f'{v*1e3:g}' for v in r['pin_diameters_m']) for r in records)+'.','',
        '|Design|Adopted stiffness / MN/m|Maximum sharing factor|Peak pressure / MPa|Minimum output stiffness / N·m/rad|Maximum pin bending / MPa|Minimum bearing static safety|Configured checks|',
        '|---|---|---:|---:|---:|---:|---:|---|']
    for row in records:
        k=' / '.join(f'{v/1e6:.3f}' for v in row['adopted_pin_stiffness_n_per_m'])
        english.append(f"|{english_labels[row['scenario']]}|{k}|{row['maximum_load_share_factor']:.6f}|"
            f"{row['maximum_contact_pressure_pa']/1e6:.3f}|{row['minimum_output_stiffness_n_m_per_rad']:.3f}|"
            f"{row['maximum_pin_bending_stress_pa']/1e6:.3f}|{row['minimum_planet_bearing_static_safety_factor']:.6f}|"
            f"{'Pass' if row['configured_checks_pass'] else 'Fail'}|")
    english+=['',f"Manual input and the geometry source produce {equivalence['compared_arrays']} elementwise-identical response arrays.",'',
        f"Each design has {len(rows)//(len(records)*count)} duty states. "
        f"{'All system/mesh couplings converged.' if all(r['all_couplings_converged'] for r in records) else 'Some system/mesh couplings did not converge.'} "
        f"Maximum output-torque residual: {max(r['maximum_torque_residual_n_m'] for r in records):.3g} N·m; "
        f"maximum path residual: {max(r['maximum_path_residual_m'] for r in records):.3g} m.",'',
        'Failed configured checks: '+('; '.join(english_labels[r['scenario']]+': '+', '.join(r['failed_configured_checks'])
            for r in records if r['failed_configured_checks']) or 'none')+'.','',
        'The short-pin beam model includes bending and transverse shear, but excludes pin-hole indentation, fits, carrier-plate deformation and local 3D contact. Bearing capacities are held fixed for this comparison; actual diameter changes require bearing reselection.',
        'Input consistency, equilibrium and convergence are numerical integration checks, not physical validation of the joint assembly.','']
    (output/'report.en.md').write_text('\n'.join(english),encoding='utf-8',newline='\n')
    names=['source_input.json','summary.json','comparison.npz','sharing.csv','report.md','report.en.md']+[name+'_input.json' for name in variants]
    write_json(output/'manifest.json',{'files':{name:hashlib.sha256((output/name).read_bytes()).hexdigest() for name in names}})
    return summary


def main():
    root=project_root();parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=root/'examples/dfki_pin_geometry.json')
    parser.add_argument('--output',type=Path,default=root/'examples/results/pin_stiffness_feedback')
    parser.add_argument('--smaller-diameter-m',type=float,default=.004)
    parser.add_argument('--raw-output',type=Path)
    args=parser.parse_args()
    if args.input.resolve().is_relative_to(args.output.resolve()):
        parser.error('结果目录不能包含或覆盖原输入文件')
    run(json.loads(args.input.read_text(encoding='utf-8')),args.output,
        smaller_diameter_m=args.smaller_diameter_m,raw_output=args.raw_output)


if __name__=='__main__':main()
