"""从对标与离散研究CSV/JSON绘制双语蓝红论文图，不重新拟合或改变结果。"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np

from robot_joint.figure_style import COLORS, publication_settings, save_publication_figure


def draw_validation(source, output, languages=('zh', 'en')):
    """绘制响应重合、逐点误差和离散敏感性；实测/总成状态由图义限定。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    def rows(name):
        with (source/name).open(encoding='utf-8-sig', newline='') as stream:
            return list(csv.DictReader(stream))
    cycle = rows('cycle.csv')
    strips = rows('strip_forces.csv')
    summary = [row for row in rows('comparison.csv') if row['group'] == 'ltca']
    grid = json.loads((source/'resolution/resolution.json').read_text(encoding='utf-8'))
    for language in languages:
        L = lambda zh, en: zh if language == 'zh' else en
        with plt.rc_context(publication_settings(language)):
            fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
            phase = np.array([float(row['phase_cycles']) for row in cycle])
            # 几何配准后的数组仍按原轮转顺序保存；显示时按模一相位排序，避免跨周期斜连。
            order = np.argsort(phase)
            for ax, field, factor, ylabel in (
                (axes[0, 0], 'te_m', 1e6, L('法向TE / μm', 'Normal TE / μm')),
                (axes[0, 1], 'stiffness_n_per_m', 1e-6, L('法向切线刚度 / N/μm', 'Normal tangent stiffness / N/μm'))):
                ax.plot(phase[order], np.array([float(row['project_'+field])*factor for row in cycle])[order],
                        color=COLORS[0], label=L('当前内核', 'Current kernel'))
                ax.plot(phase[order], np.array([float(row['reference_'+field])*factor for row in cycle])[order],
                        'o', color=COLORS[1], mfc='none', ms=4, label='MASTA')
                ax.set(xlabel=L('啮合周期相位', 'Mesh-cycle phase'), ylabel=ylabel)
                ax.legend()
                ax.grid(alpha=.15)
            labels = [L('平均TE', 'Mean TE'), L('TE峰峰值', 'TE peak-to-peak'),
                      L('端点峰压', 'Peak endpoint pressure'), L('平均刚度', 'Mean stiffness')]
            axes[1, 0].barh(labels, [float(row['relative_error'])*1e6 for row in summary], color=COLORS[0])
            axes[1, 0].set(xlabel=L('相对差 / ppm', 'Relative difference / ppm'),
                           title=L('同输入下的差异；非工程精度', 'Same-input differences; not engineering accuracy'))
            x = np.array([float(row['reference_force_n']) for row in strips])
            y = np.array([float(row['project_force_n']) for row in strips])
            axes[1, 1].scatter(x, y, color=COLORS[0], s=6, alpha=.22, label=L('5040个条带中心', '5040 strip centres'))
            axes[1, 1].plot([0, max(x)], [0, max(x)], '--', color=COLORS[1], label='y = x')
            axes[1, 1].set(xlabel=L('MASTA条带力 / N', 'MASTA strip force / N'),
                           ylabel=L('当前内核条带力 / N', 'Current strip force / N'))
            axes[1, 1].legend()
            save_publication_figure(fig, output/language/'01_masta_comparison', language=language,
                title=L('18/77斜齿单副：当前内核与已保存MASTA参照', '18/77 helical gear pair: current kernel and saved MASTA reference'),
                footnote=L('共同输入与几何配准；未运行厂商软件；不构成NGW总成或物理验证。',
                           'Common inputs and geometry registration; no vendor runtime; no whole-joint or physical validation.'))
            plt.close(fig)
            fig, axes = plt.subplots(1, 2, figsize=(12, 5.8), layout='constrained')
            data = grid['results']
            ticks = [f"{row['phases']}×{row['strips']}" for row in data]
            for key, label, color, marker in (
                ('sun_planet', L('太阳轮副', 'Sun–planet'), COLORS[0], 'o-'),
                ('planet_ring', L('内齿副', 'Planet–ring'), COLORS[1], 's--')):
                axes[0].plot(ticks, [row[key+'_te_peak_to_peak_m']*1e6 for row in data], marker, color=color, label=label)
            axes[0].set(ylabel=L('TE峰峰值 / μm', 'TE peak-to-peak / μm'),
                        title=L('粗相位采样会漏过短接触分支', 'Coarse phases can miss a short contact branch'))
            for field, label, color, marker in (
                ('planet_ring_mean_te_m', L('内齿副平均TE', 'Ring-mesh mean TE'), COLORS[0], 'o-'),
                ('output_stiffness_n_m_per_rad', L('输出刚度', 'Output stiffness'), COLORS[1], 's--'),
                ('maximum_contact_pressure_pa', L('峰压', 'Peak pressure'), COLORS[2], '^-.')):
                axes[1].plot(ticks, [row['relative_difference_from_finest'][field]*100 for row in data],
                             marker, color=color, label=label)
            axes[1].set(ylabel=L('相对最后网格的差 / %', 'Difference from final grid / %'),
                        title=L('分指标报告离散敏感性', 'Report sensitivity separately for each metric'))
            for ax in axes:
                ax.set_xlabel(L('相位数 × 每线条带数', 'Phases × strips per line'))
                ax.tick_params(axis='x', rotation=35)
                ax.grid(alpha=.15)
                ax.legend(fontsize=8)
            save_publication_figure(fig, output/language/'02_resolution', language=language,
                title=L('摆台峰压工况：真实求解网格的离散敏感性', 'Pendulum peak-pressure state: sensitivity to actual solver resolution'),
                footnote=L('每点为独立平衡求解；最后网格仅作本次参照，未证明全工况/渐近收敛。',
                           'Every point is an equilibrium solve; the final grid is a reference, not proof of global convergence.'))
            plt.close(fig)


def main():
    root = next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    parser = argparse.ArgumentParser(description='重绘中英对标与网格研究图')
    parser.add_argument('--source', type=Path, default=root/'benchmarks/results')
    parser.add_argument('--output', type=Path, default=root/'benchmarks/results')
    parser.add_argument('--language', choices=['zh', 'en', 'both'], default='both', help='图表标注语言')
    args = parser.parse_args()
    draw_validation(args.source, args.output, ('zh', 'en') if args.language == 'both' else (args.language,))


if __name__ == '__main__':
    main()
