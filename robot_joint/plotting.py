"""关节方案的工程结果图；绘图与计算分离，不改变原始数组。"""
from pathlib import Path
import math
import numpy as np
from .figure_style import COLORS, publication_settings, save_publication_figure


def plot_joint(result, output_dir):
    """输出载荷分配、重复热循环、电机工况和静态角度包络。"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from .precision import reversal_curve
    with plt.rc_context(publication_settings('zh')):
        fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
        colors = COLORS
        ax = axes[0, 0]
        n = len(result.cases)
        # 按行星画工况曲线，避免多工况分组柱越过行星坐标并产生上百条图例。
        forces = np.array([case['sharing']['tangential_forces_n'] for case in result.cases])
        for j in range(result.inputs['stage']['planet_count']):
            ax.plot(np.arange(1, n+1), forces[:, j], ['o-', 's--', '^-.'][j % 3],
                    label=f'行星{j+1}', color=colors[j % len(colors)], markersize=3)
        ax.set(xlabel='工况序号', ylabel='每副切向力 / N', title='行星载荷分配')
        ax.legend(fontsize=8)
        ax = axes[0, 1]
        time = np.asarray(result.thermal['time_s'])
        temperature = np.asarray(result.thermal['temperature_k'])-273.15
        for j, label in enumerate(('电机', '减速器')):
            ax.plot(time, temperature[:, j], color=colors[j], label=label, linewidth=2)
        ax.set(xlabel='单个任务周期时间 / s', ylabel='温度 / °C',
               title='周期热稳态' if result.summary['thermal_mode']=='periodic_steady' else '单次历程温升')
        ax.legend()
        ax = axes[1, 0]
        motor = result.inputs['motor']
        w = np.linspace(0, motor['maximum_speed_rad_s'], 200)
        for name, style, label in [('continuous_torque_n_m', '-', '连续包络'), ('peak_torque_n_m', '--', '峰值包络')]:
            cap = motor[name]*np.minimum(1, motor['base_speed_rad_s']/np.maximum(w, 1e-20))
            ax.plot(w, cap, style, label=label, color='#727982' if style=='--' else colors[0])
        for i, case in enumerate(result.motor['segments']):
            ax.scatter(abs(case['motor_speed_rad_s']), abs(case['motor_torque_n_m']), s=55,
                       color=colors[i % len(colors)], zorder=3)
        ax.set(xlabel='电机转速幅值 / rad/s', ylabel='电机转矩幅值 / N·m', title='机电匹配')
        ax.legend(fontsize=8)
        ax = axes[1, 1]
        max_t = max(abs(s['output_torque_n_m']) for s in result.inputs['duty_segments'])
        curve = reversal_curve(np.linspace(-max_t, max_t, 101),
                      output_backlash_rad=result.inputs['precision']['output_backlash_rad'],
                      torsional_stiffness_n_m_per_rad=result.summary['minimum_output_torsional_stiffness_n_m_per_rad'])
        factor = 180/math.pi*60
        for key, label, style in [('positive_envelope_rad', '正工作面包络', '-'), ('negative_envelope_rad', '负工作面包络', '--')]:
            ax.plot(curve['torques_n_m'], np.array(curve[key])*factor, style, color=colors[0], label=label)
        ax.set(xlabel='输出扭矩 / N·m', ylabel='输出转角 / arcmin', title='双工作面静态包络（非真实迟滞轨迹）')
        ax.legend(fontsize=8)
        for axis in axes.flat:
            axis.grid(alpha=.15)
        path = Path(output_dir)/'overview.png'
        save_publication_figure(fig, path, title='NGW关节设计计算 · '+result.inputs['case_id'],
                                footnote='给定参数下的准静态设计结果；不代表样机实测。')
        plt.close(fig)
