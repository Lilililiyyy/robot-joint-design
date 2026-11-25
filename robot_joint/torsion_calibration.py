"""双向静态扭矩角度标定、独立试次留验及同载荷模型对照，内部采用SI。"""
from copy import deepcopy
import math
from numbers import Real

import numpy as np

from .analysis import _output_torsion, _prepare_transmission
from .gears import coupled_planet_contacts, CouplingConvergenceError
from .support import support_response


FLANKS = ('positive', 'negative')
SAMPLE_FIELDS = ('run_id', 'flank', 'torque_n_m', 'angle_rad')


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{name}须为有限实数')
    return float(value)


def _sample_rows(samples, split, window, fixture):
    if not isinstance(samples, list) or not samples:
        raise ValueError(f'{split}数据须为非空列表')
    for index, sample in enumerate(samples, 1):
        prefix = f'{split}第{index}条'
        if not isinstance(sample, dict) or set(sample) != set(SAMPLE_FIELDS):
            raise ValueError(f'{prefix}字段须为{", ".join(SAMPLE_FIELDS)}')
        run_id, flank, torque, angle = (sample[name] for name in SAMPLE_FIELDS)
        if not isinstance(run_id, str) or not run_id.strip() or run_id != run_id.strip():
            raise ValueError(f'{prefix}试次ID须为非空且首尾无空格的字符串')
        torque = _finite(torque, prefix+'输出扭矩N·m')
        angle = _finite(angle, prefix+'读数rad')
        if flank not in FLANKS or torque == 0 or (flank == 'positive') != (torque > 0):
            raise ValueError(f'{prefix}工作面须与非零扭矩符号一致')
        if not window[0] <= abs(torque) <= window[1]:
            raise ValueError(f'{prefix}扭矩超出载荷窗口，请先选取稳定承载数据')
        corrected = _finite(angle-fixture*torque, prefix+'夹具修正角rad')
        yield {'split': split, **sample, 'torque_n_m': torque, 'angle_rad': angle,
               'corrected_angle_rad': corrected}


def _error_metrics(errors):
    residuals = np.asarray(errors, dtype=float)
    peak = float(np.max(np.abs(residuals)))
    # 先按幅值归一，避免在计算均方根时平方溢出。
    scaled = residuals/peak if peak else residuals
    return {'sample_count': len(residuals), 'bias_rad': float(scaled.mean())*peak,
            'rmse_rad': float(np.sqrt(np.mean(scaled**2)))*peak,
            'maximum_absolute_error_rad': peak}


def calibrate_torsion(fit_samples, verification_samples, *, absolute_torque_window_n_m,
                      fixture_compliance_rad_per_n_m, verification_limit_rad):
    """按工作面拟合已承载分支 θ=a+cT，计算K=1/c和留验误差。

    两批样本按试次ID隔离。输入全为输出侧量，夹具柔度另行测得或明确填0。
    留验数据不参与拟合，要求处于相同工作面的拟合扭矩范围内。
    两截距之差是向零扭矩外推的分支间隔，保留符号和零点。
    """
    window = absolute_torque_window_n_m
    if not isinstance(window, (list, tuple)) or len(window) != 2:
        raise ValueError('absolute_torque_window_n_m须给出最小和最大扭矩幅值')
    low, high = (_finite(value, '扭矩窗口N·m') for value in window)
    fixture = _finite(fixture_compliance_rad_per_n_m, '夹具柔度rad/(N·m)')
    limit = _finite(verification_limit_rad, '留验误差限值rad')
    if not 0 < low < high or fixture < 0 or limit <= 0:
        raise ValueError('要求0<扭矩窗口下限<上限、夹具柔度非负、留验限值为正')
    fit = list(_sample_rows(fit_samples, 'fit', (low, high), fixture))
    verification = list(_sample_rows(verification_samples, 'verification', (low, high), fixture))
    if {row['run_id'] for row in fit} & {row['run_id'] for row in verification}:
        raise ValueError('拟合与留验不得共享试次ID')

    branches = {}
    for flank in FLANKS:
        training = [row for row in fit if row['flank'] == flank]
        held_out = [row for row in verification if row['flank'] == flank]
        torque = np.array([row['torque_n_m'] for row in training])
        measured = np.array([row['corrected_angle_rad'] for row in training])
        if len(set(torque)) < 3 or len({row['torque_n_m'] for row in held_out}) < 2:
            raise ValueError(f'{flank}需要至少3个拟合及2个独立留验扭矩值')
        minimum, maximum = float(torque.min()), float(torque.max())
        if any(not minimum <= row['torque_n_m'] <= maximum for row in held_out):
            raise ValueError(f'{flank}留验扭矩超出该工作面拟合范围')
        # 先中心化、再以扭矩跨度归一，减小零点与斜率的数值相关。
        mean_torque = float(torque.mean())
        mean_angle = float(measured.mean())
        span = maximum-minimum
        x = (torque-mean_torque)/span
        compliance = _finite(float((x @ (measured-mean_angle))/(x @ x))/span, '拟合分支柔度')
        if compliance <= 0:
            raise ValueError(f'{flank}夹具修正后的拟合柔度必须为正，请检查符号、加载窗口和夹具标定')
        intercept = _finite(mean_angle-compliance*mean_torque, '分支零扭矩外推角')
        for row in training+held_out:
            predicted = _finite(mean_angle+compliance*(row['torque_n_m']-mean_torque), '拟合预测转角')
            row['fitted_angle_rad'] = predicted
            row['fit_residual_rad'] = _finite(row['corrected_angle_rad']-predicted, '角度残差')
        branches[flank] = {'torque_range_n_m': [minimum, maximum],
            'compliance_rad_per_n_m': compliance, 'stiffness_n_m_per_rad': _finite(1/compliance, '分支刚度'),
            'zero_torque_intercept_rad': intercept,
            'fit': _error_metrics([row['fit_residual_rad'] for row in training]),
            'verification': _error_metrics([row['fit_residual_rad'] for row in held_out])}
    gap = _finite(branches['positive']['zero_torque_intercept_rad']-
                  branches['negative']['zero_torque_intercept_rad'], '有符号分支间隔')
    verified = _error_metrics([row['fit_residual_rad'] for row in verification])
    return {'branches': branches, 'zero_torque_branch_separation_rad': gap,
            'zero_torque_branch_center_rad': branches['negative']['zero_torque_intercept_rad']+gap/2,
            'settings': {'absolute_torque_window_n_m': [low, high],
                         'fixture_compliance_rad_per_n_m': fixture, 'verification_limit_rad': limit},
            'verification': {**verified, 'within_limit': verified['maximum_absolute_error_rad'] <= limit},
            'samples': fit+verification,
            'scope': '指定承载窗口内的分支刚度和零扭矩外推间隔，留验量为修正后的输出角误差'}


def compare_torsion_model(config, calibration):
    """对纯输出扭矩逐载荷复算NGW模型，并用拟合试次对齐公共角度零点。

    输入轴和内齿圈固定，端口其他载荷为0。模型转角采用齿轮路径角、
    结构转角和有符号半侧隙。对齐只移动角度零点，不缩放刚度或改变侧隙。
    """
    stage, _, _, sharing_options, pin = _prepare_transmission(config)
    support_options = config['support']
    backlash = _finite(config['precision']['output_backlash_rad'], '模型总自由侧隙rad')
    if backlash < 0:
        raise ValueError('模型总自由侧隙不能为负')
    rows = deepcopy(calibration['samples'])
    predictions = {}
    for torque in dict.fromkeys(row['torque_n_m'] for row in rows):
        support = support_response([0, 0, 0, 0, 0, torque], **support_options)
        try:
            sharing, _ = coupled_planet_contacts(stage, torque, sharing_options, config['mesh'],
                support_displacement=support['generalized_displacement'])
        except CouplingConvergenceError as error:
            raise CouplingConvergenceError(f'静态扭矩{torque:g} N·m的模型未收敛：{error}') from error
        twist, stiffness = _output_torsion(sharing, support)
        predictions[torque] = {'model_angle_rad': twist+math.copysign(backlash/2, torque),
            'model_tangent_stiffness_n_m_per_rad': stiffness,
            'path_residual_m': sharing['coupling']['path_residual_m']}

    # 拟合组只识别同一个编码器零点，留验组不参与任何对齐。
    fit = [row for row in rows if row['split'] == 'fit']
    offset = float(np.mean([row['corrected_angle_rad']-predictions[row['torque_n_m']]['model_angle_rad']
                            for row in fit]))
    for row in rows:
        row.update(predictions[row['torque_n_m']])
        row['aligned_model_angle_rad'] = _finite(row['model_angle_rad']+offset, '对齐模型角')
        row['model_residual_rad'] = _finite(row['corrected_angle_rad']-row['aligned_model_angle_rad'], '模型角残差')
    verification = [row for row in rows if row['split'] == 'verification']
    return {'case_id': config['case_id'], 'common_zero_offset_rad': offset,
            'model_output_backlash_rad': backlash, 'adopted_pin_stiffness_n_per_m': pin['adopted_pin_stiffness_n_per_m'],
            'evaluated_torque_count': len(predictions), 'verification': _error_metrics(
                [row['model_residual_rad'] for row in verification]),
            'verification_by_flank': {flank: _error_metrics(
                [row['model_residual_rad'] for row in verification if row['flank'] == flank]) for flank in FLANKS},
            'samples': rows}
