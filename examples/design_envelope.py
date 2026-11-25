"""使用可编辑的有限装配/支承情景，重算完整任务并导出约束裕量。"""
import argparse
import json
from pathlib import Path
import tempfile

from robot_joint import analyze_joint_envelope, export_joint_envelope


def run(config, specification, output):
    """全部计算与导出成功后再替换正式文件，保留先前完整结果。"""
    study = analyze_joint_envelope(config, specification['scenarios'],
        required_checks=specification['required_checks'])
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.design_envelope_', dir=output.parent) as directory:
        folder = export_joint_envelope(study, directory)
        output.mkdir(parents=True, exist_ok=True)
        paths = [p for p in folder.iterdir() if p.name != 'manifest.json']+[folder/'manifest.json']
        for path in paths:
            path.replace(output/path.name)
    return study


def main():
    root = next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, default=root/'examples/dfki_pin_geometry.json')
    parser.add_argument('--scenarios', type=Path, default=root/'examples/dfki_assembly_scenarios.json')
    parser.add_argument('--output', type=Path, default=root/'examples/results/design_envelope')
    args = parser.parse_args()
    if any(p.resolve().is_relative_to(args.output.resolve()) for p in (args.input, args.scenarios)):
        parser.error('输出目录不能包含原输入或情景定义文件')
    study = run(json.loads(args.input.read_text(encoding='utf-8')),
        json.loads(args.scenarios.read_text(encoding='utf-8')), args.output)
    print(json.dumps({'情景数': study['evaluated_scenario_count'],
        '全部列出情景通过': study['all_scenarios_accepted'], '结果目录': str(args.output)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
