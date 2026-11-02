"""检查峰压代表工况的相位/条带离散敏感性；不将有限网格比较冒称全域收敛。"""
from __future__ import annotations
import argparse
from copy import deepcopy
import csv
import json
from pathlib import Path
import time

from robot_joint import analyze_joint
from gear_contact.io import write_json


def run_resolution_study(base, case_index):
    """固定一个真实配置工况，先加密相位再加密条带，保存求解结果而非插值。"""
    rows = []
    for phases, strips in ((16, 13), (32, 13), (64, 13), (64, 26), (64, 52), (128, 26), (256, 26)):
        config = deepcopy(base)
        config['duty_segments'] = [deepcopy(base['duty_segments'][case_index])]
        config['mesh'].update(phases=phases, strips=strips)
        # 单状态用于离散比较；不把完整任务的热点谱或周期热设计用于该状态验收。
        config.pop('fatigue', None)
        config.pop('design_limits', None)
        config['case_id'] = f"resolution_{phases}_{strips}"
        begin = time.perf_counter()
        result = analyze_joint(config)
        case = result.cases[0]
        row = {'phases': phases, 'strips': strips,
               'maximum_contact_pressure_pa': result.summary['maximum_contact_pressure_pa'],
               'load_share_factor': case['sharing']['load_share_factor'],
               'output_stiffness_n_m_per_rad': result.summary['minimum_output_torsional_stiffness_n_m_per_rad'],
               'path_residual_m': case['sharing']['coupling']['path_residual_m'],
               'converged': case['sharing']['coupling']['converged'],
               'elapsed_s': time.perf_counter()-begin}
        for key, group in case['contacts'].items():
            row[key+'_mean_te_m'] = max(a.summary['mean_normal_approach_m'] for a in group)
            row[key+'_te_peak_to_peak_m'] = max(a.summary['te_peak_to_peak_m'] for a in group)
        rows.append(row)
    reference = rows[-1]
    fields = ('maximum_contact_pressure_pa', 'load_share_factor', 'output_stiffness_n_m_per_rad',
              'sun_planet_mean_te_m', 'planet_ring_mean_te_m')
    for row in rows:
        row['relative_difference_from_finest'] = {key: abs(row[key]-reference[key])/abs(reference[key]) for key in fields}
    return {'case_index_zero_based': case_index,
            'case_label': base['duty_segments'][case_index]['label'],
            'reference_resolution': [256, 26], 'results': rows,
            'scope_zh': '仅对先前基准结果中的峰压工况检查离散敏感性；最终网格只是当前比较基准，未证明渐近收敛，未扫描所有工况和参数。',
            'physical_validation': False}


def main():
    root = next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    parser = argparse.ArgumentParser(description='峰压工况的相位与条带离散敏感性')
    parser.add_argument('--output', type=Path, default=root/'benchmarks/results/resolution')
    args = parser.parse_args()
    base = json.loads((root/'examples/dfki_pendulum_case.json').read_text(encoding='utf-8'))
    metrics = json.loads((root/'examples/results/dfki_pendulum/metrics.json').read_text(encoding='utf-8'))
    data = run_resolution_study(base, metrics['peak_case_index'])
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output/'resolution.json', data)
    rows = [{k: v for k, v in row.items() if k != 'relative_difference_from_finest'} for row in data['results']]
    with (args.output/'resolution.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(data, ensure_ascii=False))


if __name__ == '__main__':
    main()
