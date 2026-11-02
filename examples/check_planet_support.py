"""从完整关节结果读取销反力，复算两组明确假设的销与行星轴承方案。"""
from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import hashlib
import json
from pathlib import Path

from robot_joint import analyze_joint
from robot_joint.planet_support import assess_planet_support
from gear_contact.io import write_json


def project_root():
    """按独立项目标记定位，不依赖启动目录。"""
    return next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def load_exported_duty(directory):
    """核对导出清单后读取原销反力/转速，既有数组和哈希不被修改。"""
    folder = Path(directory)
    manifest = read(folder/'manifest.json')
    names = ['input.json']
    base = read(folder/'input.json')
    cases = []
    for j, segment in enumerate(base['duty_segments']):
        stem = f'case_{j+1:03d}/'
        names.extend([stem+'sharing.json', stem+'speeds.json'])
        cases.append({'index': j, 'label': segment.get('label', str(j+1)),
                      'sharing': read(folder/(stem+'sharing.json')),
                      'speeds': read(folder/(stem+'speeds.json'))})
    hashes = {}
    for name in names:
        digest = hashlib.sha256((folder/name).read_bytes()).hexdigest()
        if manifest['files'].get(name) != digest:
            raise ValueError('已导出关节输入/载荷文件与原SHA-256清单不符：'+name)
        hashes[name] = digest
    return base, cases, {'source': 'exported_joint_result', 'export_version': manifest['version'],
                         'manifest_verified': True, 'source_files_sha256': hashes}


def default_assumptions():
    """演示尺寸、材料及容量均为新增设计假设，不是DFKI/mjbots实物参数。"""
    pin = {'diameter_m': .004, 'support_span_m': .012, 'loaded_width_m': .005,
           'young_modulus_pa': 210e9, 'poisson_ratio': .3, 'shear_coefficient': .9,
           'allowable_bending_stress_pa': 300e6, 'allowable_shear_stress_pa': 180e6,
           'allowable_projected_pressure_pa': 60e6, 'required_safety_factor': 1.5}
    larger = deepcopy(pin)
    larger['diameter_m'] = .006
    bearing = {'dynamic_capacity_n': 800., 'static_capacity_n': 500., 'exponent': 10/3,
               'radial_factor': 1., 'axial_factor': 0., 'static_radial_factor': 1.,
               'static_axial_factor': 0.}
    return {'candidates': [{'id': 'pin_4mm', 'pins': pin, 'bearing_ratings': bearing},
                           {'id': 'pin_6mm', 'pins': larger, 'bearing_ratings': bearing}],
            'status_zh': '新增设计假设；直径/跨距/材料许用值/剪切系数和轴承容量均非原产品规格，未选真实采购型号。',
            'status_en': 'New design assumptions: dimensions, allowables, shear factor and bearing capacities are not original product specifications; no catalog part is selected.',
            'comparison_zh': '仅改变销径，保持已收敛载荷、支承跨距和演示轴承容量相同；现实中改销径须重选轴承并反馈重算。',
            'comparison_en': 'Only pin diameter changes; solved loads, support span and illustrative bearing capacities are held fixed. A real diameter change requires bearing selection and feedback into load sharing.'}


def run(base, cases, assumptions, provenance, output):
    """保存逐工况结果及同一载荷下的候选对照，不重复运行昂贵LTCA。"""
    folder = Path(output)
    folder.mkdir(parents=True, exist_ok=True)
    candidates, rows, history = [], [], []
    candidate_options = assumptions.get('candidates')
    if not isinstance(candidate_options, list) or not candidate_options:
        raise ValueError('设计假设须给出非空candidates列表')
    used_ids = set()
    for candidate in candidate_options:
        name = candidate['id']
        if not isinstance(name, str) or not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name) or name in used_ids:
            raise ValueError('候选id须为唯一的字母/数字/下划线/连字符组合')
        used_ids.add(name)
        result = assess_planet_support(cases, base['duty_segments'], pins=candidate['pins'],
            bearing_ratings=candidate['bearing_ratings'],
            specified_pin_stiffness_n_per_m=base['load_sharing']['pin_stiffness_n_per_m'])
        write_json(folder/(name+'.json'), result)
        candidates.append({'id': name, 'pins': result['pins'], 'summary': result['summary'],
                           'planet_bearings': result['planet_bearings']})
        for pin, bearing in zip(result['pins'], result['planet_bearings']):
            rows.append({'candidate_id': name, **pin,
                         'basic_bearing_l10_hours': bearing['life_hours'],
                         'bearing_static_safety_factor': bearing['minimum_static_safety_factor']})
        for state in result['cases']:
            j = state['index']
            for i, pin in enumerate(state['pins']):
                history.append({'candidate_id': name, 'case_index': j, 'label': state['label'], 'planet_index': i,
                    'duration_s': base['duty_segments'][j]['duration_s'],
                    'relative_speed_rad_s': cases[j]['speeds']['planet_relative_to_carrier_rad_s'],
                    'pin_load_n': pin['load_n'], 'bending_stress_pa': pin['bending_stress_pa'],
                    'transverse_shear_stress_pa': pin['maximum_transverse_shear_stress_pa'],
                    'projected_average_pressure_pa': pin['projected_average_pressure_pa'],
                    'center_deflection_m': pin['center_deflection_m'],
                    'load_weighted_deflection_m': pin['load_weighted_deflection_m'],
                    'static_checks_pass': pin['static_checks_pass']})
    for filename, values in (('comparison.csv', rows), ('history.csv', history)):
        with (folder/filename).open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(values[0]))
            writer.writeheader()
            writer.writerows(values)
    summary = {'case_id': base['case_id'], 'duty_states': len(cases),
               'assumptions': assumptions, 'provenance': provenance, 'candidates': candidates,
               'system_stiffness_was_updated': False, 'physical_validation': False,
               'scope_zh': '实际分载结果下的销静强度/弯剪挠度及行星轴承基本L10。未重新分载，未验证短粗销三维接触、摆动磨损、预紧、润滑或实物寿命。',
               'scope_en': 'Pin static strength/bending-shear deflection and planet-bearing basic L10 under solved loads. No load-sharing feedback, 3D short-pin contact, oscillatory wear, preload, lubrication or hardware-life validation.'}
    write_json(folder/'summary.json', summary)
    return summary


def main():
    root = project_root()
    parser = argparse.ArgumentParser(description='给定销几何与轴承评级，校核已收敛关节销反力')
    parser.add_argument('--results', type=Path, help='已有export_joint完整导出目录，校验SHA清单后直接读取')
    parser.add_argument('--input', type=Path, help='无已有结果时使用的关节配置，默认DFKI摆台案例')
    parser.add_argument('--assumptions', type=Path, help='可选的自定义候选销和轴承参数JSON')
    parser.add_argument('--output', type=Path, default=root/'examples/results/planet_support')
    args = parser.parse_args()
    if args.results:
        base, cases, provenance = load_exported_duty(args.results)
        if args.input and read(args.input) != base:
            raise ValueError('--input与已有结果的原始输入不一致')
    else:
        source = args.input or root/'examples/dfki_pendulum_case.json'
        base = read(source)
        result = analyze_joint(base)
        cases = result.cases
        provenance = {'source': 'analyze_joint', 'manifest_verified': False,
                      'input_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}
    assumptions = read(args.assumptions) if args.assumptions else default_assumptions()
    result = run(base, cases, assumptions, provenance, args.output)
    print(json.dumps({'duty_states': result['duty_states'], 'candidates': len(result['candidates']),
                      'system_stiffness_was_updated': False, 'physical_validation': False}, ensure_ascii=False))


if __name__ == '__main__':
    main()
