"""新写的输出角度预算与双工作面静态包络；角度rad、刚度N·m/rad。

确定性上界逐项相加，不把相关或确定性误差按统计RSS合成。这里的自由
侧隙与线弹性不能预测真实摩擦迟滞、微动、控制补偿或编码器标定误差。
"""
from __future__ import annotations

import math
from numbers import Integral, Real


def _number(value, name, *, minimum=None, strict=False):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name}必须为有限实数")
    result = float(value)
    if minimum is not None and (result <= minimum if strict else result < minimum):
        raise ValueError(f"{name}超出允许范围")
    return result


def precision_budget(*, output_elastic_twist_rad,
                     output_torsional_stiffness_n_m_per_rad, output_backlash_rad,
                     encoder_counts_per_revolution, encoder_on_output=True,
                     ratio=1.0, assembly_error_rad=0.0):
    """给出指定受载状态的确定性输出角度预算。

    output_backlash_rad为输出端两个工作面之间的总自由转角，名义中心
    到单侧预算取其一半。输入侧编码器的半计数角再除以ratio；输出侧
    编码器不除速比。本函数不自动把弹性角视为闭环剩余误差，也不推断
    编码器准确度、齿距误差或多级轮系误差，调用方须单独给出装配预算。
    """
    twist = _number(output_elastic_twist_rad, "output_elastic_twist_rad")
    stiffness = _number(output_torsional_stiffness_n_m_per_rad,
                        "output_torsional_stiffness_n_m_per_rad", minimum=0, strict=True)
    backlash = _number(output_backlash_rad, "output_backlash_rad", minimum=0)
    assembly = _number(assembly_error_rad, "assembly_error_rad")
    ratio = _number(ratio, "ratio", minimum=0, strict=True)
    if (isinstance(encoder_counts_per_revolution, bool)
            or not isinstance(encoder_counts_per_revolution, Integral)
            or encoder_counts_per_revolution < 2):
        raise ValueError("编码器每转计数必须为不小于2的整数")
    if not isinstance(encoder_on_output, bool):
        raise ValueError("encoder_on_output必须为布尔值")
    encoder_step = 2*math.pi/int(encoder_counts_per_revolution)
    if not encoder_on_output:
        encoder_step /= ratio
    half_count = encoder_step/2
    total = abs(twist)+backlash/2+half_count+abs(assembly)
    return {
        "model": "deterministic_output_angle_budget", "implementation": "新写的确定性误差预算",
        "output_elastic_twist_rad": twist,
        "output_torsional_stiffness_n_m_per_rad": stiffness,
        "output_backlash_rad": backlash, "backlash_half_width_rad": backlash/2,
        "encoder_counts_per_revolution": int(encoder_counts_per_revolution),
        "encoder_on_output": encoder_on_output,
        "encoder_output_step_rad": encoder_step,
        "encoder_output_half_count_rad": half_count,
        "assembly_error_rad": assembly,
        "deterministic_worst_case_error_rad": total,
        "scope": "名义中心参考的指定受载状态上界；未模拟真实摩擦迟滞、标定误差或控制补偿",
        "physical_validation": False,
    }


def reversal_curve(torques_n_m, *, output_backlash_rad,
                   torsional_stiffness_n_m_per_rad, friction_torque_n_m=0.0):
    """给定扭矩处的双工作面静态角度包络，不是历史相关的迟滞轨迹。

    两包络为θ±=(T±Tf)/K±b/2；Tf是调用方给定的等效库仑摩擦界限。
    该式仅给出自由侧隙、线性弹性及给定摩擦界限的外包络；它不选择
    当前工作面，不模拟接触切换，不含摩擦粘滑或非线性预紧。
    """
    backlash = _number(output_backlash_rad, "output_backlash_rad", minimum=0)
    stiffness = _number(torsional_stiffness_n_m_per_rad,
                        "torsional_stiffness_n_m_per_rad", minimum=0, strict=True)
    friction = _number(friction_torque_n_m, "friction_torque_n_m", minimum=0)
    if isinstance(torques_n_m, (str, bytes)):
        raise ValueError("torques_n_m必须为非空扭矩序列")
    try:
        torques = [_number(value, "torques_n_m") for value in torques_n_m]
    except TypeError as error:
        raise ValueError("torques_n_m必须为非空扭矩序列") from error
    if not torques:
        raise ValueError("torques_n_m必须为非空扭矩序列")
    return {
        "model": "two_flank_static_envelope", "history_dependent": False,
        "torques_n_m": torques,
        "positive_envelope_rad": [(value+friction)/stiffness+backlash/2 for value in torques],
        "negative_envelope_rad": [(value-friction)/stiffness-backlash/2 for value in torques],
        "output_backlash_rad": backlash,
        "torsional_stiffness_n_m_per_rad": stiffness,
        "friction_torque_n_m": friction,
        "scope": "双工作面静态外包络；不表示按输入顺序积分得到的迟滞或回差实测曲线",
        "physical_validation": False,
    }
