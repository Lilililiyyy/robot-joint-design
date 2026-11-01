"""新写的单级NGW运动学、柔度凝聚与单边行星载荷分配。

采用固定内齿圈、太阳轮输入、行星架输出、零变位直齿和小变形假设。
几何长度m，路径切向力N，扭矩N·m；不是厂商完整系统求解器的恢复。
输入的路径柔度已凝聚到每颗行星的切向传力坐标，不重复计入LTCA。
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from gear_contact.solver import solve_contact


def _finite_scalar(value, name: str, *, positive: bool = False) -> float:
    """校验SI标量，布尔值不能被当作数值参数。"""
    if (isinstance(value, (bool, np.bool_))
            or not isinstance(value, (int, float, np.integer, np.floating))):
        raise ValueError(f"{name}必须为有限实数")
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}必须为有限实数") from error
    if not math.isfinite(result) or (positive and result <= 0):
        raise ValueError(f"{name}必须为{'正' if positive else ''}有限实数")
    return result


@dataclass(frozen=True)
class PlanetaryStage:
    """均布直齿NGW几何；所有齿数取正，内齿圈符号由其角色确定。"""

    sun_teeth: int
    planet_teeth: int
    ring_teeth: int
    planet_count: int
    module_m: float
    face_width_m: float
    pressure_angle_rad: float = math.radians(20.0)

    def __post_init__(self) -> None:
        for name in ("sun_teeth", "planet_teeth", "ring_teeth"):
            value = getattr(self, name)
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 3:
                raise ValueError(f"{name}必须为不小于3的整数")
        if (isinstance(self.planet_count, (bool, np.bool_))
                or not isinstance(self.planet_count, (int, np.integer)) or self.planet_count < 2):
            raise ValueError("planet_count必须为不小于2的整数")
        _finite_scalar(self.module_m, "模数module_m", positive=True)
        _finite_scalar(self.face_width_m, "齿宽face_width_m", positive=True)
        angle = _finite_scalar(self.pressure_angle_rad, "压力角pressure_angle_rad")
        if not 0 < angle < math.pi / 2:
            raise ValueError("压力角必须在0与π/2之间，单位rad")
        if self.ring_teeth != self.sun_teeth + 2 * self.planet_teeth:
            raise ValueError("零变位同模数NGW必须满足ring_teeth=sun_teeth+2*planet_teeth")
        if (self.sun_teeth + self.ring_teeth) % self.planet_count:
            raise ValueError("均布行星装配必须满足(sun_teeth+ring_teeth)/planet_count为整数")
        radius = self.module_m * (self.sun_teeth + self.planet_teeth) / 2
        neighbour_spacing = 2 * radius * math.sin(math.pi / self.planet_count)
        planet_tip_diameter = self.module_m * (self.planet_teeth + 2)
        if neighbour_spacing <= planet_tip_diameter:
            raise ValueError("相邻行星齿顶圆相交或相切，当前均布布局不可用")


def stage_geometry(stage: PlanetaryStage) -> dict:
    """返回参考圆、均布布局和装配检查；不冒称刀具根切/齿廓干涉校核。"""
    if not isinstance(stage, PlanetaryStage):
        raise ValueError("stage必须为PlanetaryStage")
    sun = stage.module_m * stage.sun_teeth / 2
    planet = stage.module_m * stage.planet_teeth / 2
    ring = stage.module_m * stage.ring_teeth / 2
    radius = sun + planet
    angles = 2 * math.pi * np.arange(stage.planet_count, dtype=float) / stage.planet_count
    return {
        "ratio": 1 + stage.ring_teeth / stage.sun_teeth,
        "sun_radius_m": sun,
        "planet_radius_m": planet,
        "ring_radius_m": ring,
        "carrier_radius_m": radius,
        "planet_angles_rad": angles,
        "planet_centres_xy_m": radius * np.column_stack((np.cos(angles), np.sin(angles))),
        "assembly_index": (stage.sun_teeth + stage.ring_teeth) // stage.planet_count,
        "adjacent_tip_clearance_m": (2 * radius * math.sin(math.pi / stage.planet_count)
                                     - stage.module_m * (stage.planet_teeth + 2)),
        "geometry_scope": "零变位直齿参考几何、均布装配及邻接齿顶圆检查；未检查刀具根切和完整齿廓干涉",
    }


def speed_map(stage: PlanetaryStage, output_speed_rad_s: float) -> dict:
    """Willis运动学；行星自转分别返回惯性坐标绝对值与行星架相对值。"""
    speed = _finite_scalar(output_speed_rad_s, "输出角速度")
    geometry = stage_geometry(stage)
    sun_speed = geometry["ratio"] * speed
    planet_relative = -stage.sun_teeth / stage.planet_teeth * (sun_speed - speed)
    return {
        "sun_rad_s": sun_speed,
        "ring_rad_s": 0.0,
        "carrier_rad_s": speed,
        "planet_rad_s": planet_relative + speed,
        "planet_relative_to_carrier_rad_s": planet_relative,
        "ratio": geometry["ratio"],
    }


def _positive_vector(value, size: int, name: str) -> np.ndarray:
    """允许标量或逐行星刚度；所有值均为正有限N/m。"""
    raw = np.asarray(value)
    if raw.dtype.kind == "b":
        raise ValueError(f"{name}不能使用布尔值")
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}必须为正有限刚度，单位N/m") from error
    if array.ndim == 0:
        array = np.full(size, float(array))
    if array.shape != (size,) or np.any(~np.isfinite(array)) or np.any(array <= 0):
        raise ValueError(f"{name}必须为正有限标量或长度为{size}的数组，单位N/m")
    return array


def _symmetric_matrix(value, shape: tuple[int, int], name: str, *, definite: bool) -> np.ndarray:
    """按弹性互易性检查矩阵；刚度须正定，附加柔度允许半正定。"""
    raw = np.asarray(value)
    if raw.dtype.kind == "b":
        raise ValueError(f"{name}不能使用布尔值")
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name}必须为有限实矩阵") from error
    if matrix.shape != shape or not np.all(np.isfinite(matrix)):
        raise ValueError(f"{name}必须为{shape[0]}×{shape[1]}有限矩阵")
    scale = float(np.max(np.abs(matrix)))
    if not np.allclose(matrix, matrix.T, rtol=1e-11, atol=scale * 1e-13):
        raise ValueError(f"{name}必须对称以满足弹性互易性")
    matrix = (matrix + matrix.T) / 2
    eigenvalues = np.linalg.eigvalsh(matrix)
    if (definite and eigenvalues[0] <= 0) or (not definite and eigenvalues[0] < -scale * 1e-11):
        raise ValueError(f"{name}必须{'正定' if definite else '半正定'}")
    return matrix


def solve_load_sharing(
    stage: PlanetaryStage,
    output_torque_n_m: float,
    *,
    sun_mesh_stiffness_n_per_m,
    ring_mesh_stiffness_n_per_m,
    pin_stiffness_n_per_m,
    sun_radial_stiffness_n_per_m,
    path_gaps_m=None,
    extra_compliance_m_per_n=None,
) -> dict:
    """由输出扭矩求各行星力、太阳平移及输出端等效切线刚度。

    ks、kr是两啮合副法向刚度N/m；kpin是行星销切向支承刚度N/m。
    路径未知力ft是每副参考圆切向力幅值，Fn=ft/cos(alpha)。销受2*ft，
    因此其互补能对路径产生4/kpin柔度。路径间隙是所选工作面两副
    合并后的切向等效间隙，正值推迟该路径接触，单位m。

    太阳轮二维平移采用弹簧刚度K。D_j=t_j+s*tan(alpha)*r_j，s是
    传递扭矩方向；共享平移柔度为D*K^-1*D.T。太阳实际受力为
    -s*D.T*ft。附加柔度应已经凝聚到相同路径坐标，不能再次包括
    上述齿面、销或太阳支承柔度。

    输出弹性角为传动理想位置到承载位置的滞后角，含路径间隙基准；
    T=0时该角约定为0，path_approach仍保留首次接触位置min(gaps)。
    此处不求所有构件六自由度位移、动态惯性、摩擦或完整轮体FE。
    """
    torque = _finite_scalar(output_torque_n_m, "输出扭矩")
    geometry = stage_geometry(stage)
    count = stage.planet_count
    ks = _positive_vector(sun_mesh_stiffness_n_per_m, count, "太阳轮副法向刚度")
    kr = _positive_vector(ring_mesh_stiffness_n_per_m, count, "内齿圈副法向刚度")
    kp = _positive_vector(pin_stiffness_n_per_m, count, "行星销切向刚度")
    sun_raw = np.asarray(sun_radial_stiffness_n_per_m)
    if sun_raw.ndim == 0:
        sun_scalar = _finite_scalar(sun_radial_stiffness_n_per_m, "太阳轮径向刚度", positive=True)
        sun_k = sun_scalar * np.eye(2)
    else:
        sun_k = _symmetric_matrix(sun_radial_stiffness_n_per_m, (2, 2), "太阳轮径向刚度", definite=True)
    gaps = np.zeros(count) if path_gaps_m is None else np.asarray(path_gaps_m, dtype=float)
    if gaps.shape != (count,) or not np.all(np.isfinite(gaps)):
        raise ValueError(f"路径间隙必须为长度{count}的有限数组，单位m")
    extra = (np.zeros((count, count)) if extra_compliance_m_per_n is None else
             _symmetric_matrix(extra_compliance_m_per_n, (count, count), "附加路径柔度", definite=False))
    # 无载约定使用正工作面的右切线，不据此声称处于确定的承载齿面。
    direction = 1 if torque >= 0 else -1
    angles = geometry["planet_angles_rad"]
    radial = np.column_stack((np.cos(angles), np.sin(angles)))
    tangential = np.column_stack((-np.sin(angles), np.cos(angles)))
    alpha = stage.pressure_angle_rad
    projection = tangential + direction * math.tan(alpha) * radial
    cos2 = math.cos(alpha) ** 2
    local_compliance = 1 / (ks * cos2) + 1 / (kr * cos2) + 4 / kp
    sun_compliance = projection @ np.linalg.solve(sun_k, projection.T)
    compliance = np.diag(local_compliance) + sun_compliance + extra
    lever = 2 * geometry["carrier_radius_m"]
    total_force = abs(torque) / lever
    solution = solve_contact(gaps, compliance, total_force)
    force = solution.forces_n
    sum_force = float(np.sum(force))
    sun_force = -direction * projection.T @ force
    ring_projection = tangential - direction * math.tan(alpha) * radial
    ring_force = -direction * ring_projection.T @ force
    sun_translation = np.linalg.solve(sun_k, sun_force)
    carrier_forces = direction * 2 * force[:, None] * tangential
    output_reconstructed = direction * lever * sum_force
    sun_torque = direction * geometry["sun_radius_m"] * sum_force
    ring_torque = direction * geometry["ring_radius_m"] * sum_force
    diagnostics = dict(solution.diagnostics)
    diagnostics.update({
        "output_torque_balance_n_m": output_reconstructed - torque,
        "external_torque_balance_n_m": sun_torque + ring_torque - output_reconstructed,
        "external_force_balance_xy_n": sun_force + ring_force + np.sum(carrier_forces, axis=0),
        "minimum_compliance_eigenvalue_m_per_n": float(np.linalg.eigvalsh(compliance)[0]),
        "active_planets": int(np.count_nonzero(solution.active_mask)),
        "zero_load_angle_convention": "无载时输出滞后角约定为0；路径接近量保留首次接触基准",
        "model": "新写单级直齿NGW互补能降阶模型；正定支承、单边路径接触",
        "physical_validation": False,
        "vendor_validation": False,
    })
    return {
        "geometry": geometry,
        "direction": direction,
        "tangential_forces_n": force,
        "normal_forces_n": force / math.cos(alpha),
        "carrier_pin_forces_xy_n": carrier_forces,
        "sun_force_xy_n": sun_force,
        "ring_force_xy_n": ring_force,
        "sun_translation_m": sun_translation,
        "path_approach_m": float(solution.approach_m),
        "path_tangent_stiffness_n_per_m": float(solution.tangent_stiffness_n_per_m),
        "output_torsional_stiffness_n_m_per_rad": float(lever**2 * solution.tangent_stiffness_n_per_m),
        "output_elastic_twist_rad": direction * float(solution.approach_m) / lever if torque else 0.0,
        "load_share_factor": count * float(np.max(force)) / sum_force if sum_force else None,
        "active_mask": solution.active_mask,
        "path_residual_gaps_m": solution.residual_gaps_m,
        "compliance_m_per_n": compliance,
        "effective_path_gaps_m": gaps.copy(),
        "sun_input_torque_n_m": sun_torque,
        "ring_reaction_torque_n_m": ring_torque,
        "carrier_output_torque_n_m": output_reconstructed,
        "diagnostics": diagnostics,
    }
