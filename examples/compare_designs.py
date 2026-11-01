"""保持几何与外部评级因子一致，比较三种显式支承/误差候选。"""
import argparse
import json
from pathlib import Path

from robot_joint import compare_joint_designs
from gear_contact.io import write_json
from .pendulum_case import candidate_configs


def main():
    parser = argparse.ArgumentParser(description='NGW关节有限候选方案比较')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    base = json.loads(Path(__file__).with_name('dfki_pendulum_case.json').read_text(encoding='utf-8'))
    study = compare_joint_designs(candidate_configs(base))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, {k:v for k,v in study.items() if k != 'results'})
    print(str(args.output))


if __name__ == '__main__':
    main()
