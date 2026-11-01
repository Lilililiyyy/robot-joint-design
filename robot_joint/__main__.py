"""读取关节配置并导出可复算结果，不执行配置中的代码。"""
import argparse
import json
from pathlib import Path

from .analysis import analyze_joint
from .io import export_joint


def main():
    parser = argparse.ArgumentParser(description='单级NGW机器人关节模组设计计算')
    parser.add_argument('input', type=Path, help='SI单位JSON输入')
    parser.add_argument('--output', type=Path, required=True, help='结果目录')
    parser.add_argument('--plots', action='store_true', help='生成中文性能图')
    args = parser.parse_args()
    try:
        result = analyze_joint(json.loads(args.input.read_text(encoding='utf-8')))
        export_joint(result, args.output, plots=args.plots)
    except (ValueError, KeyError, TypeError, RuntimeError, OSError) as error:
        parser.exit(2, f'计算失败：{error}\n')
    print(json.dumps({'case_id': result.summary['case_id'], 'output': str(args.output),
                      'configured_checks_pass': result.summary['configured_checks_pass']}, ensure_ascii=False))


if __name__ == '__main__':
    main()
