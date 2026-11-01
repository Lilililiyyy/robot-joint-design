"""保持几何与外部评级因子一致，比较三种显式支承/误差候选。"""
import argparse
import copy
import json
from pathlib import Path

from robot_joint import compare_joint_designs
from gear_contact.io import write_json


def main():
    parser = argparse.ArgumentParser(description='NGW关节有限候选方案比较')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    base = json.loads(Path(__file__).with_name('ngw_18_27_72.json').read_text(encoding='utf-8'))
    candidates = [copy.deepcopy(base) for _ in range(3)]
    candidates[1]['case_id'] = 'ngw_larger_path_errors'
    for key in ('positive_path_gaps_m', 'negative_path_gaps_m'):
        candidates[1]['load_sharing'][key] = [3*v for v in candidates[1]['load_sharing'][key]]
    candidates[2]['case_id'] = 'ngw_softer_pins'
    candidates[2]['load_sharing']['pin_stiffness_n_per_m'] = [v*.3 for v in candidates[2]['load_sharing']['pin_stiffness_n_per_m']]
    study = compare_joint_designs(candidates)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, {k:v for k,v in study.items() if k != 'results'})
    print(str(args.output))


if __name__ == '__main__':
    main()
