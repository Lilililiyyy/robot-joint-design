"""新写的凝聚支承响应、双支承静力与基本额定寿命计算。

支承矩阵由调用方提供，不在这里恢复轴承内部接触、预紧或壳体有限元。
全部采用SI：力N、力矩N·m、位移m、转角rad、时间s、角速度rad/s。
"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np


def _scalar(value, name, *, positive=False, nonnegative=False):
    """拒绝布尔、非数值及非有限参数，避免单位配置被隐式转换。"""
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(float(value)):
        raise ValueError(f"{name}必须为有限实数")
    number = float(value)
    if positive and number <= 0:
        raise ValueError(f"{name}必须大于0")
    if nonnegative and number < 0:
        raise ValueError(f"{name}不能为负")
    return number


def _array(value, name, shape=None):
    """只接受有限实数数组；shape省略时要求非空一维序列。"""
    raw = np.asarray(value)
    if raw.dtype.kind not in "iuf":
        raise ValueError(f"{name}必须为实数数组")
    result = raw.astype(float)
    if (shape is not None and result.shape != shape) or (shape is None and (result.ndim != 1 or result.size == 0)):
        raise ValueError(f"{name}维度不符")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name}不能含NaN或无穷值")
    return result


def support_response(wrench, stiffness_matrix, *, reference_offset_m=0.0) -> dict:
    """由凝聚6×6刚度及参考点偏置求输出载荷端口小位移。

    W=[Fx,Fy,Fz,Mx,My,Mz]，u=[x,y,z,θx,θy,θz]。
    K的平移块为N/m，旋转块为N·m/rad，耦合项须满足虚功互易。
    reference_offset_m是K参考点到输出载荷端口的有符号z距离h。
    uport=B uref，B的平移—转动块为-[r]×；Wref=Bᵀ Wport，故
    Mref=Mport+r×F。先在K参考点解K uref=Wref，再返回uport，
    两端虚功完全一致。h缺省0，与历史的端口凝聚矩阵接口兼容。
    此偏置独立于bearings.load_offset_m（后者原点为双支承中点）。
    刚度可由梁、试验或FE缩聚取得；本函数不补设缺失的约束。
    """
    force = _array(wrench, "六分量载荷", (6,))
    offset = _scalar(reference_offset_m, "结构刚度参考点到载荷端口的z偏置m")
    transform = np.eye(6)
    transform[0, 4], transform[1, 3] = offset, -offset
    reference_force = transform.T @ force
    stiffness = _array(stiffness_matrix, "凝聚支承刚度", (6, 6))
    diagonal = np.diag(stiffness)
    if np.any(diagonal <= 0):
        raise ValueError("凝聚支承刚度必须正定，不能含未约束刚体自由度")
    scale = np.sqrt(diagonal)
    normalized = stiffness / np.outer(scale, scale)
    if not np.allclose(normalized, normalized.T, rtol=1e-10, atol=1e-12):
        raise ValueError("凝聚支承刚度必须对称并满足弹性互易")
    normalized = (normalized + normalized.T) / 2
    try:
        np.linalg.cholesky(normalized)
    except np.linalg.LinAlgError as error:
        raise ValueError("凝聚支承刚度必须正定，不能含未约束刚体自由度") from error
    reference_displacement = np.linalg.solve(normalized, reference_force / scale) / scale
    displacement = transform @ reference_displacement
    residual = stiffness @ reference_displacement - reference_force
    denominator = max(float(np.linalg.norm(reference_force / scale)), np.finfo(float).tiny)
    reference_compliance = (np.linalg.solve(normalized, np.eye(6)) / np.outer(scale, scale))
    port_compliance = transform @ reference_compliance @ transform.T
    return {
        "generalized_displacement": displacement.tolist(),
        "displacement_m": displacement[:3].tolist(),
        "rotation_rad": displacement[3:].tolist(),
        "reference_offset_m": offset,
        "reference_wrench": reference_force.tolist(),
        "reference_generalized_displacement": reference_displacement.tolist(),
        "load_point_compliance": port_compliance.tolist(),
        "strain_energy_j": float(np.dot(force, displacement) / 2),
        "diagnostics": {
            "wrench_residual": residual.tolist(),
            "relative_scaled_residual": float(np.linalg.norm(residual / scale) / denominator),
            "stiffness_positive_definite": True,
            "virtual_work_residual_j": float(np.dot(force, displacement)
                                              - np.dot(reference_force, reference_displacement)),
        },
        "model": "给定凝聚刚度下的线弹性小位移响应",
        "physical_validation": False,
    }


def two_bearing_reactions(wrench, spacing_m, load_offset_m=0.0, axial_share=0.5) -> dict:
    """求两径向支承对外部力和倾覆矩的静力反力。

    两支承位于z=-L/2、+L/2，外力作用于z=load_offset_m；wrench中的
    力矩是该作用点的自由力矩。返回的是轴承作用于轴的反力。
    axial_share为负z支承承担轴向力的比例，由定位/浮动布置显式给定。
    Mz由传动链传递，不分配为轴承径向反力，也不在本模型求预紧载荷。
    """
    load = _array(wrench, "六分量载荷", (6,))
    length = _scalar(spacing_m, "支承跨距m", positive=True)
    offset = _scalar(load_offset_m, "载荷轴向偏置m")
    share = _scalar(axial_share, "轴向载荷分配比例")
    if not 0 <= share <= 1:
        raise ValueError("轴向载荷分配比例必须在0和1之间")
    force, moment = load[:3], load[3:]
    total_moment = moment + np.cross([0.0, 0.0, offset], force)
    reactions = np.empty((2, 3))
    reactions[0] = [-force[0] / 2 + total_moment[1] / length,
                    -force[1] / 2 - total_moment[0] / length,
                    -share * force[2]]
    reactions[1] = [-force[0] / 2 - total_moment[1] / length,
                    -force[1] / 2 + total_moment[0] / length,
                    -(1 - share) * force[2]]
    reaction_moment = (np.cross([0.0, 0.0, -length / 2], reactions[0])
                       + np.cross([0.0, 0.0, length / 2], reactions[1]))
    return {
        "reaction_vectors_n": reactions.tolist(),
        "radial_loads_n": np.linalg.norm(reactions[:, :2], axis=1).tolist(),
        "axial_loads_n": np.abs(reactions[:, 2]).tolist(),
        "diagnostics": {
            "force_residual_n": (reactions.sum(axis=0) + force).tolist(),
            "bending_moment_residual_n_m": (reaction_moment + total_moment)[:2].tolist(),
            "axial_torque_carried_by_drivetrain_n_m": float(moment[2]),
        },
        "model": "双径向支承静力分配；轴向分担比例外部给定，绕轴扭矩由传动链承担",
        "physical_validation": False,
    }


def bearing_duty_life(radial_loads_n, axial_loads_n, speeds_rad_s, durations_s, *,
                      dynamic_capacity_n, static_capacity_n, exponent=3.0,
                      radial_factor=1.0, axial_factor=1.0,
                      static_radial_factor=1.0, static_axial_factor=1.0) -> dict:
    """对单个轴承累计一组恒载片段的基本L10损伤与静安全系数。

    P=X Fr+Y Fa，P0=X0 Fr+Y0 Fa；X/Y必须对应实际轴承目录的当前分支。
    不自动查表或判断Fa/Fr阈值，跨分支工况应由调用方预先等效。
    n_i=|ω_i| Δt_i/(2π)，D=Σ(P_i/C)^p n_i/10^6；重复历程L10为1/D。
    p=3对应球轴承，p=10/3对应滚子轴承。该基本额定寿命不含润滑、污染、
    预紧、可靠度修正、微动损伤及小角度摆动修正，零速段仅评价静载。
    """
    radial = _array(radial_loads_n, "径向载荷N")
    axial = _array(axial_loads_n, "轴向载荷N", radial.shape)
    speeds = _array(speeds_rad_s, "转速rad/s", radial.shape)
    duration = _array(durations_s, "持续时间s", radial.shape)
    if np.any(radial < 0) or np.any(axial < 0) or np.any(duration <= 0):
        raise ValueError("径向/轴向载荷必须非负，各工况持续时间必须大于0")
    capacity = _scalar(dynamic_capacity_n, "基本额定动载荷N", positive=True)
    static_capacity = _scalar(static_capacity_n, "基本额定静载荷N", positive=True)
    power = _scalar(exponent, "寿命指数", positive=True)
    x = _scalar(radial_factor, "径向动载系数", nonnegative=True)
    y = _scalar(axial_factor, "轴向动载系数", nonnegative=True)
    x0 = _scalar(static_radial_factor, "径向静载系数", nonnegative=True)
    y0 = _scalar(static_axial_factor, "轴向静载系数", nonnegative=True)
    if x + y == 0 or x0 + y0 == 0:
        raise ValueError("动载和静载系数不能分别同时为0")
    equivalent = x * radial + y * axial
    static_equivalent = x0 * radial + y0 * axial
    revolutions = np.abs(speeds) * duration / (2 * math.pi)
    with np.errstate(over="ignore", invalid="ignore"):
        individual_damage = (equivalent / capacity) ** power * revolutions / 1e6
    if not np.all(np.isfinite(individual_damage)) or not np.all(np.isfinite(revolutions)):
        raise ValueError("载荷、转数或寿命指数超过可计算数值范围")
    damage = float(individual_damage.sum())
    total_revolutions = float(revolutions.sum())
    if total_revolutions > 0:
        scale = float(np.max(equivalent))
        equivalent_dynamic = (scale * float(np.dot((equivalent / scale) ** power, revolutions)
                                             / total_revolutions) ** (1 / power)) if scale > 0 else 0.0
    else:
        equivalent_dynamic = None
    repeats = 1 / damage if damage > 0 else None
    maximum_static = float(np.max(static_equivalent))
    return {
        "dynamic_equivalent_load_n": equivalent.tolist(),
        "static_equivalent_load_n": static_equivalent.tolist(),
        "revolutions_per_segment": revolutions.tolist(),
        "damage_per_segment": individual_damage.tolist(),
        "damage_per_duty": damage,
        "repeats_to_l10": repeats,
        "life_hours": repeats * float(duration.sum()) / 3600 if repeats is not None else None,
        "equivalent_dynamic_load_n": equivalent_dynamic,
        "minimum_static_safety_factor": static_capacity / maximum_static if maximum_static > 0 else None,
        "diagnostics": {
            "zero_rotation_duty": total_revolutions == 0,
            "total_revolutions": total_revolutions,
            "zero_damage": damage == 0,
            "none_life_meaning": "无转动疲劳损伤时寿命返回None，不表示微动、静压痕或实物寿命无限",
        },
        "method": "L10基本额定寿命与变载线性损伤累计",
        "physical_validation": False,
    }
