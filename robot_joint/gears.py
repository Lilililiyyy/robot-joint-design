"""复用公开圆柱齿轮内核的NGW齿面适配，保留原单位与计算结果。"""
from __future__ import annotations

import math
import numpy as np

from gear_contact import (Gear, GearPair, Material, analyze_pair, modification_from_config,
                          MeshRatingFactors, GearRatingFactors, StressLimits,
                          evaluate_iso6336_explicit_factors)
from .planetary import stage_geometry, solve_load_sharing


class CouplingConvergenceError(RuntimeError):
    """载荷、齿面变形与系统平衡未同时满足声明容差。"""


def gear_pairs(stage):
    """构造太阳轮为小轮的外啮合，以及行星轮为小轮的内啮合。"""
    common = dict(normal_module_m=stage.module_m, face_width_m=stage.face_width_m,
                  pressure_angle_rad=stage.pressure_angle_rad)
    sun = Gear(teeth=stage.sun_teeth, **common)
    planet = Gear(teeth=stage.planet_teeth, **common)
    ring = Gear(teeth=stage.ring_teeth, internal=True, **common)
    return {'sun_planet': GearPair(sun, planet), 'planet_ring': GearPair(planet, ring)}


def _planet_vector(value, count, name):
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.full(count, float(array))
    if array.shape != (count,) or not np.all(np.isfinite(array)):
        raise ValueError(name + '必须为有限标量或每颗行星的数组')
    return array


def _mesh_analysis(pair, force_t, options, member_mods, misalignment):
    material = Material(**options['material'])
    torque = float(force_t * pair.pinion.reference_radius_m)
    mode = options.get('elastic_mode', 'iso')
    if mode not in ('iso', 'iso_love', 'coupled', 'diagonal'):
        raise ValueError('elastic_mode必须为iso、iso_love、coupled或diagonal')
    kwargs = dict(torque_n_m=torque, material_pinion=material, material_wheel=material,
                  modification_pinion=modification_from_config(member_mods.get('pinion', {})),
                  modification_wheel=modification_from_config(member_mods.get('wheel', {})),
                  misalignment_m=float(misalignment), phases=options.get('phases', 12),
                  strips=options.get('strips', 9), elastic_mode=mode,
                  local_contact=mode not in ('iso', 'iso_love'))
    if mode in ('iso', 'iso_love'):
        kwargs['iso_settings'] = {'nominal_pinion_torque_n_m': torque,
                                 **options.get('iso_settings', {})}
        if kwargs['iso_settings']['nominal_pinion_torque_n_m'] != torque:
            raise ValueError('关节适配层名义扭矩由实际局部载荷计算，不允许重复覆盖')
    return analyze_pair(pair, **kwargs)


def coupled_planet_contacts(stage, torque, sharing, mesh, *, support_displacement=None):
    """求周期平均LTCA与凝聚系统的载荷—位移协调。

    每副Fn=Ft/cosα，路径变形=δn/cosα。使用局部切线与截距
    g0=δn/cosα−Ft/(kn*cos²α)构造下一次系统线性化；不将切线
    刚度错误地当作非线性割线。模型不求同步啮合相位的整机动态响应。
    """
    pairs = gear_pairs(stage)
    n = stage.planet_count
    c = math.cos(stage.pressure_angle_rad)
    direction = 'positive' if torque >= 0 else 'negative'
    gaps = _planet_vector(sharing.get(direction + '_path_gaps_m', 0), n, '路径间隙')
    base = dict(pin_stiffness_n_per_m=sharing['pin_stiffness_n_per_m'],
                sun_radial_stiffness_n_per_m=sharing['sun_radial_stiffness_n_per_m'],
                extra_compliance_m_per_n=sharing.get('extra_compliance_m_per_n'))
    stiffness_s = _planet_vector(sharing['initial_mesh_stiffness_n_per_m'], n, '初始啮合刚度')
    stiffness_r = stiffness_s.copy()
    if np.any(stiffness_s <= 0):
        raise ValueError('初始啮合刚度必须为正')
    modifiers = mesh['modifications'][direction]
    offsets = np.zeros(n)
    settings = mesh.get('coupling', {})
    max_iterations = settings.get('maximum_iterations', 60)
    tolerance = settings.get('force_relative_tolerance', 1e-5)
    gap_tolerance = settings.get('path_tolerance_m', 2e-9)
    relaxation = settings.get('relaxation', 0.7)
    if (isinstance(max_iterations, bool) or not isinstance(max_iterations, int) or max_iterations < 1
            or not np.isfinite(tolerance) or tolerance <= 0 or not np.isfinite(gap_tolerance)
            or gap_tolerance <= 0 or not np.isfinite(relaxation) or not 0 < relaxation <= 1):
        raise ValueError('耦合迭代上限、容差和松弛参数无效')
    misalignments = {}
    displacement = np.zeros(6) if support_displacement is None else np.asarray(support_displacement, dtype=float)
    if displacement.shape != (6,) or not np.all(np.isfinite(displacement)):
        raise ValueError('输出端结构位移必须为6分量有限数组')
    for key in pairs:
        misalignments[key] = _planet_vector(mesh.get(key + '_misalignment_m', 0), n, '齿宽失准')
        alignment = mesh.get('alignment_maps', {}).get(key)
        if alignment is not None:
            matrix = np.asarray(alignment, dtype=float)
            if matrix.shape != (n, 6) or not np.all(np.isfinite(matrix)):
                raise ValueError('失准传递矩阵必须为行星数×6的有限数组')
            misalignments[key] = misalignments[key] + matrix @ displacement
    previous = None
    history = []
    for iteration in range(1, max_iterations + 1):
        result = solve_load_sharing(stage, float(torque), sun_mesh_stiffness_n_per_m=stiffness_s,
                                    ring_mesh_stiffness_n_per_m=stiffness_r,
                                    path_gaps_m=gaps + offsets, **base)
        forces = np.asarray(result['tangential_forces_n'])
        contacts = {key: [] for key in pairs}
        new_stiffness = {key: np.zeros(n) for key in pairs}
        mesh_path = np.zeros(n)
        for key, pair in pairs.items():
            for j, force in enumerate(forces):
                analysis = _mesh_analysis(pair, force, mesh, modifiers[key], misalignments[key][j])
                contacts[key].append(analysis)
                # 均值位移对力的导数是各相位柔度的均值，因此取刚度调和平均。
                phase_k = np.asarray(analysis.mesh_stiffness_n_per_m)
                if np.any(~np.isfinite(phase_k)) or np.any(phase_k <= 0):
                    raise CouplingConvergenceError('LTCA相位刚度必须为正有限值')
                new_stiffness[key][j] = 1 / float(np.mean(1 / phase_k))
                mesh_path[j] += analysis.summary['mean_normal_approach_m'] / c
        ks, kr = new_stiffness['sun_planet'], new_stiffness['planet_ring']
        if np.any(~np.isfinite(ks)) or np.any(ks <= 0) or np.any(~np.isfinite(kr)) or np.any(kr <= 0):
            raise CouplingConvergenceError('LTCA返回非正或非有限周期平均刚度')
        old_mesh_c = 1 / (stiffness_s * c*c) + 1 / (stiffness_r * c*c)
        structural_c = np.asarray(result['compliance_m_per_n']) - np.diag(old_mesh_c)
        actual_path = gaps + mesh_path + structural_c @ forces
        loaded = forces > 0
        approach = float(result['path_approach_m'])
        residual = max(float(np.max(np.abs(actual_path[loaded] - approach))) if np.any(loaded) else 0.0,
                       float(np.max(np.maximum(approach - actual_path[~loaded], 0))) if np.any(~loaded) else 0.0)
        relative_change = (float(np.max(np.abs(forces-previous))) / max(float(np.sum(forces)), np.finfo(float).tiny)
                           if previous is not None else None)
        history.append({'iteration': iteration, 'path_residual_m': residual,
                        'force_relative_change': relative_change})
        if (torque == 0 or (relative_change is not None and relative_change <= tolerance and residual <= gap_tolerance)):
            # 最终刚度按最终接触切线组装；活动集合保持最终受力状态。
            final_c = structural_c + np.diag(1/(ks*c*c) + 1/(kr*c*c))
            final_gaps = gaps + mesh_path - forces*(1/(ks*c*c) + 1/(kr*c*c))
            if torque == 0:
                result = solve_load_sharing(stage, 0.0, sun_mesh_stiffness_n_per_m=ks,
                            ring_mesh_stiffness_n_per_m=kr, path_gaps_m=final_gaps, **base)
                approach = result['path_approach_m']
                residual = float(np.max(np.maximum(approach-actual_path, 0)))
                history[-1]['path_residual_m'] = residual
            if np.any(loaded):
                active_c = final_c[np.ix_(loaded, loaded)]
                tangent = float(np.sum(np.linalg.solve(active_c, np.ones(np.count_nonzero(loaded)))))
                result['path_tangent_stiffness_n_per_m'] = tangent
                result['output_torsional_stiffness_n_m_per_rad'] = (2*stage_geometry(stage)['carrier_radius_m'])**2*tangent
            result['compliance_m_per_n'] = final_c
            result['effective_path_gaps_m'] = final_gaps
            result['path_residual_gaps_m'] = actual_path - approach
            result['diagnostics']['active_gap_error_m'] = (float(np.max(np.abs(actual_path[loaded]-approach)))
                                                          if np.any(loaded) else 0.0)
            result['diagnostics']['maximum_penetration_m'] = float(np.max(np.maximum(approach-actual_path, 0)))
            result['coupling'] = {'converged': True, 'iterations': iteration,
                                  'path_residual_m': residual, 'history': history,
                                  'method': '周期平均LTCA切线/截距与凝聚柔度的位移协调',
                                  'tangent_convention': '各相位柔度平均，冻结最终局部名义评级参数；不是重新评级的工况差分',
                                  'misalignment_m': misalignments}
            return result, contacts
        previous = forces.copy()
        new_offset = mesh_path - forces*(1/(ks*c*c) + 1/(kr*c*c))
        stiffness_s = 1 / ((1-relaxation)/stiffness_s + relaxation/ks)
        stiffness_r = 1 / ((1-relaxation)/stiffness_r + relaxation/kr)
        offsets = (1-relaxation)*offsets + relaxation*new_offset
    raise CouplingConvergenceError(f'NGW齿面/系统协调在{max_iterations}次迭代后仍未收敛，路径残差{residual:.6g}m')


def rate_mesh(pair, force_t, material_options, options):
    """按当前局部名义载荷调用显式因子应力链，因子与限值均为外部输入。"""
    material = Material(**material_options)
    return evaluate_iso6336_explicit_factors(
        pair, nominal_pinion_torque_n_m=float(force_t*pair.pinion.reference_radius_m),
        face_width_m=pair.face_overlap_m, mesh_factors=MeshRatingFactors(**options['mesh_factors']),
        pinion_factors=GearRatingFactors(**options['pinion_factors']),
        wheel_factors=GearRatingFactors(**options['wheel_factors']),
        pinion_limits=StressLimits(**options['pinion_limits']),
        wheel_limits=StressLimits(**options['wheel_limits']),
        material_pinion=material, material_wheel=material)
