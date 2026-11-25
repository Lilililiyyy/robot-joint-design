"""读取双向静态试验CSV，输出拟合参数、独立留验及可选模型对照。"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import tempfile

from . import __version__
from .torsion_calibration import SAMPLE_FIELDS, calibrate_torsion, compare_torsion_model


def _read_samples(content):
    reader = csv.DictReader(io.StringIO(content.decode('utf-8-sig'), newline=''))
    if reader.fieldnames != list(SAMPLE_FIELDS):
        raise ValueError(f'CSV列名与顺序须为{", ".join(SAMPLE_FIELDS)}')
    samples = []
    for row in reader:
        try:
            if None in row or any(row[key] is None for key in SAMPLE_FIELDS):
                raise ValueError('列数与表头不一致')
            for name in ('torque_n_m', 'angle_rad'):
                row[name] = float(row[name])
        except (ValueError, TypeError) as error:
            raise ValueError(f'CSV第{reader.line_num}行无法读取，扭矩和角度须为SI数值') from error
        samples.append(row)
    return samples


def _write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n',
                    encoding='utf-8', newline='\n')


def _plot(samples, kind, stem, language, limit, has_model):
    import matplotlib as mpl
    import matplotlib.pyplot as plt
    from .figure_style import BLUE, RED, publication_settings, save_publication_figure

    english = language == 'en'
    with mpl.rc_context(publication_settings(language)):
        fig, (curve, residual) = plt.subplots(1, 2, figsize=(10.5, 4.0),
            width_ratios=[1.28, 1], layout='constrained')
        try:
            for flank, color in (('positive', BLUE), ('negative', RED)):
                fit = sorted((row for row in samples if row['split']=='fit' and row['flank']==flank),
                             key=lambda row: row['torque_n_m'])
                verify = [row for row in samples if row['split']=='verification' and row['flank']==flank]
                fit_x = [row['torque_n_m'] for row in fit]
                verify_x = [row['torque_n_m'] for row in verify]
                curve.plot(fit_x, [1000*row['fitted_angle_rad'] for row in fit], color=color)
                curve.plot(fit_x, [1000*row['corrected_angle_rad'] for row in fit],
                           linestyle='none', marker='o', mfc='white', color=color,
                           label=('Fit samples' if english else '拟合样本') if flank=='positive' else None)
                curve.plot(verify_x, [1000*row['corrected_angle_rad'] for row in verify],
                           linestyle='none', marker='s', color=color,
                           label=('Held-out samples' if english else '独立留验') if flank=='positive' else None)
                residual.plot(verify_x, [1e6*row['fit_residual_rad'] for row in verify],
                              linestyle='none', marker='s', color=color,
                              label=('Branch fit' if english else '分支拟合') if flank=='positive' else None)
                if has_model:
                    model_rows = sorted(fit+verify, key=lambda row: row['torque_n_m'])
                    curve.plot([row['torque_n_m'] for row in model_rows],
                               [1000*row['aligned_model_angle_rad'] for row in model_rows],
                               linestyle='--', color=color,
                               label=('NGW model' if english else 'NGW模型') if flank=='positive' else None)
            residual.axhline(0, color='#555555', linewidth=.65)
            for value in (-limit, limit):
                residual.axhline(1e6*value, color='#8b8b8b', linestyle='--', linewidth=.75)
            curve.set_ylabel('Corrected output angle (mrad)' if english else '修正输出角（mrad）')
            residual.set_ylabel('Held-out angle residual (µrad)' if english else '留验角度残差（µrad）')
            for axis in (curve, residual):
                axis.set_xlabel('Output torque (N·m)' if english else '输出扭矩（N·m）')
                if axis is curve:
                    axis.legend(loc='best')
                axis.grid(axis='y', alpha=.16)
            if kind == 'synthetic':
                title = 'Synthetic torsion example' if english else '双向扭转标定 · 合成示例'
            else:
                title = 'Bidirectional torsion calibration' if english else '双向扭转标定'
            note = ('Right panel: branch-fit residuals and verification limits.' if english
                    else '右图为分支拟合残差，灰色虚线表示留验限值。')
            save_publication_figure(fig, stem, title=title, footnote=note, language=language)
        finally:
            plt.close(fig)


def run(fit_file, verification_file, settings_file, output, *, model_file=None, plots=None):
    """按原输入字节记录来源哈希，完成计算与绘图后再写入正式目录。"""
    sources = {'fit': Path(fit_file), 'verification': Path(verification_file),
               'settings': Path(settings_file)}
    if model_file is not None:
        sources['model'] = Path(model_file)
    content = {name: path.read_bytes() for name, path in sources.items()}
    settings = json.loads(content['settings'])
    required = {'data_kind', 'source', 'absolute_torque_window_n_m',
                'fixture_compliance_rad_per_n_m', 'verification_limit_rad'}
    if not isinstance(settings, dict) or set(settings) != required:
        raise ValueError('设置须包含数据类别、来源、扭矩窗口、夹具柔度和留验限值')
    kind, source = settings['data_kind'], settings['source']
    if kind not in ('synthetic', 'measured') or not isinstance(source, str) or not source.strip():
        raise ValueError('data_kind须为synthetic或measured，并给出非空的source记录')
    options = {key: value for key, value in settings.items() if key not in ('data_kind', 'source')}
    result = calibrate_torsion(_read_samples(content['fit']), _read_samples(content['verification']), **options)
    model = compare_torsion_model(json.loads(content['model']), result) if 'model' in content else None
    samples = model['samples'] if model is not None else result['samples']
    report = {key: value for key, value in result.items() if key != 'samples'}
    report.update(data_kind=kind, source=source,
        model_comparison={key: value for key, value in model.items() if key != 'samples'} if model is not None else None)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.torsion_', dir=output.parent) as temporary:
        draft = Path(temporary)
        _write_json(draft/'calibration.json', report)
        with (draft/'samples.csv').open('w', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(samples[0]), lineterminator='\n')
            writer.writeheader()
            writer.writerows(samples)
        languages = ('zh', 'en') if plots == 'both' else (plots,) if plots else ()
        for language in languages:
            _plot(samples, kind, draft/('torsion_'+language), language,
                  result['settings']['verification_limit_rad'], model is not None)
        _write_json(draft/'manifest.json', {'version': __version__,
            'inputs': {name: {'file': path.name, 'sha256': hashlib.sha256(content[name]).hexdigest()}
                       for name, path in sources.items()},
            'files': {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in sorted(draft.iterdir())}})
        output.mkdir(parents=True, exist_ok=True)
        for path in sorted(draft.iterdir(), key=lambda path: path.name == 'manifest.json'):
            path.replace(output/path.name)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fit', type=Path, required=True, help='拟合试次CSV')
    parser.add_argument('--verification', type=Path, required=True, help='独立留验试次CSV')
    parser.add_argument('--settings', type=Path, required=True, help='来源、夹具柔度、承载窗口和限值JSON')
    parser.add_argument('--model', type=Path, help='用于纯输出扭矩对照的关节设计JSON')
    parser.add_argument('--output', type=Path, required=True, help='结果目录')
    parser.add_argument('--plots', choices=('zh', 'en', 'both'), help='输出蓝红色扭转图')
    args = parser.parse_args()
    if any(path is not None and path.resolve().is_relative_to(args.output.resolve())
           for path in (args.fit, args.verification, args.settings, args.model)):
        parser.error('结果目录须与输入目录分开')
    try:
        result = run(args.fit, args.verification, args.settings, args.output,
                     model_file=args.model, plots=args.plots)
    except (ValueError, KeyError, TypeError, RuntimeError, OSError, ImportError) as error:
        parser.exit(2, f'标定失败：{error}\n')
    passed = result['verification']['within_limit']
    print(json.dumps({'留验通过': passed, '结果目录': str(args.output)}, ensure_ascii=False))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
