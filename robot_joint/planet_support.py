"""新写行星销简支梁静强度、弯剪挠度与行星轴承基本评级初筛。

销校核使用最终横向反力；显式刚度来源解析可在分载前从销参数取得kpin。
全部内部量采用SI，不将中心最大挠度的倒数用作系统广义刚度。
"""
from __future__ import annotations

from copy import deepcopy
import math
from numbers import Real
import numpy as np

from .support import bearing_duty_life


def _scalar(value, name, *, positive=False):
    """校验有限实数；拒绝布尔及隐式字符串转换。"""
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(float(value)):
        raise ValueError(name+'必须为有限实数')
    value = float(value)
    if positive and value <= 0:
        raise ValueError(name+'必须大于0')
    return value


def _forces(value):
    """销轴沿z，横向载荷为有符号的[Fx,Fy]，单位N。"""
    raw = np.asarray(value)
    if raw.dtype.kind not in 'iuf' or raw.shape != (2,) or not np.all(np.isfinite(raw)):
        raise ValueError('行星销横向载荷必须为有限实数二分量[Fx,Fy]，单位N')
    return raw.astype(float)


def pin_response(force_xy_n, *, diameter_m, support_span_m, loaded_width_m,
                 young_modulus_pa, poisson_ratio, shear_coefficient,
                 allowable_bending_stress_pa, allowable_shear_stress_pa,
                 allowable_projected_pressure_pa, required_safety_factor=1.0,
                 support_model='simply_supported_centered_uniform') -> dict:
    """计算圆实销，两端简支、中心宽w内均布的横向载荷F。

    I=πd⁴/64，A=πd²/4，G=E/[2(1+ν)]。弯曲及横剪切能量分别积分，
    给出载荷共轭平均柔度与中心最大挠度；等效kpin采用前者的倒数。
    最大弯曲、最大横剪切和平均投影面压分别校核，不拼成伪同点应力。
    未包含销孔/支承局部压陷、过盈、轴向力、扭转载荷或圆角应力集中。
    """
    if support_model != 'simply_supported_centered_uniform':
        raise ValueError('仅支持两端简支、中心均布加载；其他支承/非均布载荷尚未实现')
    force = _forces(force_xy_n)
    d = _scalar(diameter_m, '销直径m', positive=True)
    length = _scalar(support_span_m, '简支跨距m', positive=True)
    width = _scalar(loaded_width_m, '加载宽度m', positive=True)
    elastic = _scalar(young_modulus_pa, '弹性模量Pa', positive=True)
    nu = _scalar(poisson_ratio, '泊松比')
    kappa = _scalar(shear_coefficient, '剪切修正系数', positive=True)
    required = _scalar(required_safety_factor, '要求安全系数', positive=True)
    if width > length:
        raise ValueError('加载宽度不能超过两端简支跨距')
    if not -1 < nu < .5:
        raise ValueError('各向同性泊松比必须位于-1与0.5之间')
    if kappa > 1:
        raise ValueError('本接口的有效剪切面积比shear_coefficient必须不大于1')
    if required < 1:
        raise ValueError('要求安全系数不能小于1')
    limits = {
        'bending': _scalar(allowable_bending_stress_pa, '许用弯曲应力Pa', positive=True),
        'shear': _scalar(allowable_shear_stress_pa, '许用横剪应力Pa', positive=True),
        'projected_pressure': _scalar(allowable_projected_pressure_pa, '许用投影平均面压Pa', positive=True),
    }
    try:
        area = math.pi*d*d/4
        inertia = math.pi*d**4/64
    except OverflowError as error:
        raise ValueError('销截面参数超过浮点计算范围') from error
    if not all(math.isfinite(v) and v > 0 for v in (area, inertia)):
        raise ValueError('销截面参数超过浮点计算范围')
    shear_modulus = elastic/(2*(1+nu))
    magnitude = math.hypot(float(force[0]), float(force[1]))
    try:
        moment = magnitude*(2*length-width)/8
        shear = magnitude/2
        bending_average = (5*length**3-5*length*width**2+2*width**3)/(240*elastic*inertia)
        shear_average = (length/4-width/6)/(kappa*shear_modulus*area)
        bending_center = (8*length**3-4*length*width**2+width**3)/(384*elastic*inertia)
        shear_center = (2*length-width)/(8*kappa*shear_modulus*area)
        average_compliance = bending_average+shear_average
        center_compliance = bending_center+shear_center
        stresses = {'bending': 32*moment/(math.pi*d**3),
                    'shear': 4*shear/(3*area),
                    'projected_pressure': magnitude/(d*width)}
    except (OverflowError, ZeroDivisionError) as error:
        raise ValueError('销载荷、材料或几何超过浮点计算范围') from error
    if not all(math.isfinite(v) for v in (magnitude, moment, shear_modulus, average_compliance,
               center_compliance, *stresses.values(), magnitude*center_compliance,
               magnitude*magnitude*average_compliance)) or min(average_compliance, center_compliance) <= 0:
        raise ValueError('销载荷、挠度或应力超过浮点计算范围')
    checks = {key: {'actual_pa': value, 'allowable_pa': limits[key],
                    'required_safety_factor': required,
                    'safety_factor': limits[key]/value if value else None,
                    'status': 'passed' if value*required <= limits[key] else 'failed'}
              for key, value in stresses.items()}
    return {
        'force_xy_n': force.tolist(), 'load_n': magnitude,
        'support_reactions_xy_n': (-np.tile(force/2, (2, 1))).tolist(),
        'maximum_bending_moment_n_m': moment, 'maximum_transverse_shear_n': shear,
        'bending_stress_pa': stresses['bending'], 'maximum_transverse_shear_stress_pa': stresses['shear'],
        'projected_average_pressure_pa': stresses['projected_pressure'],
        'center_deflection_m': magnitude*center_compliance,
        'load_weighted_deflection_m': magnitude*average_compliance,
        'center_displacement_xy_m': (force*center_compliance).tolist(),
        'load_weighted_displacement_xy_m': (force*average_compliance).tolist(),
        'load_weighted_bending_compliance_m_per_n': bending_average,
        'load_weighted_shear_compliance_m_per_n': shear_average,
        'center_bending_compliance_m_per_n': bending_center,
        'center_shear_compliance_m_per_n': shear_center,
        'equivalent_pin_stiffness_n_per_m': 1/average_compliance,
        'section_area_m2': area, 'second_moment_m4': inertia, 'shear_modulus_pa': shear_modulus,
        'strain_energy_j': .5*magnitude*magnitude*average_compliance,
        'checks': checks, 'static_checks_pass': all(v['status']=='passed' for v in checks.values()),
        'diagnostics': {'force_residual_xy_n': (force-force/2-force/2).tolist(),
                        'span_to_diameter_ratio': length/d,
                        'short_pin_review_required': length/d < 10,
                        'short_pin_criterion': 'L/d<10是本项目保守提示，非标准验收界限；已计横剪柔度仍不能替代三维销孔接触'},
        'model': '圆实销两端简支、中心均布横载；线弹性弯曲与Timoshenko横剪柔度',
        'physical_validation': False,
    }


def _per_planet(value, count, name):
    if isinstance(value, dict):
        return [deepcopy(value) for _ in range(count)]
    if not isinstance(value, list) or len(value) != count or not all(isinstance(v, dict) for v in value):
        raise ValueError(name+'须为统一参数对象或逐行星参数列表')
    return deepcopy(value)


def resolve_pin_stiffness(planet_count, sharing, *, pins=None):
    """解析系统采用的逐销切向刚度及来源，不修改输入或求解任何载荷。

    缺省specified沿用显式标量/逐销数组；pin_geometry采用已有销参数与
    pin_response零载下的平均弯剪柔度。该线性柔度与载荷幅值无关；零载
    只用来提取现有闭式公式，并非用零载替代最终强度/轴承校核。
    """
    if isinstance(planet_count, bool) or not isinstance(planet_count, int) or planet_count < 1:
        raise ValueError('行星数必须为正整数')
    if not isinstance(sharing, dict):
        raise ValueError('load_sharing必须为参数对象')
    source = sharing.get('pin_stiffness_source', 'specified')
    if not isinstance(source, str) or source not in ('specified', 'pin_geometry'):
        raise ValueError('pin_stiffness_source仅支持specified或pin_geometry')
    specified = deepcopy(sharing.get('pin_stiffness_n_per_m'))
    records = []
    if source == 'specified' or 'pin_stiffness_n_per_m' in sharing:
        raw = np.asarray(specified)
        if raw.dtype.kind not in 'iuf':
            raise ValueError('显式销刚度必须为正有限实数')
        adopted = np.asarray(raw, dtype=float)
        if adopted.ndim == 0:
            adopted = np.full(planet_count, float(adopted))
        if adopted.shape != (planet_count,) or not np.all(np.isfinite(adopted)) or np.any(adopted <= 0):
            raise ValueError('显式销刚度须为正有限标量或与行星数一致的数组')
    if source == 'specified':
        records = [{'planet_index': j, 'adopted_pin_stiffness_n_per_m': float(value),
                    'source_parameters': None, 'short_pin_review_required': None}
                   for j, value in enumerate(adopted)]
    else:
        options = _per_planet(pins, planet_count, '几何刚度模式的销参数')
        geometry_fields = ('diameter_m', 'support_span_m', 'loaded_width_m', 'young_modulus_pa',
                           'poisson_ratio', 'shear_coefficient')
        adopted = []
        for j, parameters in enumerate(options):
            try:
                response = pin_response([0.0, 0.0], **parameters)
            except TypeError as error:
                raise ValueError(f'行星{j+1}销参数包含未知字段或缺少必要参数') from error
            value = response['equivalent_pin_stiffness_n_per_m']
            _scalar(value, f'行星{j+1}采用销刚度N/m', positive=True)
            adopted.append(value)
            records.append({'planet_index': j, 'adopted_pin_stiffness_n_per_m': value,
                'source_parameters': {**{key: parameters[key] for key in geometry_fields},
                    'support_model': parameters.get('support_model', 'simply_supported_centered_uniform')},
                'load_weighted_bending_compliance_m_per_n': response['load_weighted_bending_compliance_m_per_n'],
                'load_weighted_shear_compliance_m_per_n': response['load_weighted_shear_compliance_m_per_n'],
                'span_to_diameter_ratio': response['diagnostics']['span_to_diameter_ratio'],
                'short_pin_review_required': response['diagnostics']['short_pin_review_required']})
    return {'source': source, 'adopted_pin_stiffness_n_per_m': list(map(float, adopted)),
            'specified_input_n_per_m': specified, 'specified_input_used': source == 'specified',
            'load_independent': True, 'planets': records,
            'path_compliance_contribution_m_per_n': [4/float(k) for k in adopted],
            'physical_validation': False,
            'scope': ('显式销切向刚度；采用值直接进入既有路径柔度4/kpin' if source == 'specified' else
                '简支圆实销中央均布横载的功共轭平均弯剪刚度；销力=2Ft，路径柔度=4/kpin；未计销孔/配合/架板局部柔度')}


def assess_planet_support(cases, duty_segments, *, pins, bearing_ratings,
                          specified_pin_stiffness_n_per_m=None) -> dict:
    """用已收敛的逐工况销反力校核销与行星轴承，不回写系统刚度。

    行星轴承输入转速为行星相对行星架转速，不能用输出/行星绝对转速。
    当前直齿NGW销载荷为横向二分量；轴承轴向载荷显式取0。
    bearing_ratings中X/Y等目录分支由调用者给定，不自动选择旋转圈系数。
    """
    if not isinstance(cases, list) or not cases or len(cases) != len(duty_segments):
        raise ValueError('逐工况结果与载荷片段须为非空等长列表')
    first = np.asarray(cases[0]['sharing']['carrier_pin_forces_xy_n'])
    if first.ndim != 2 or first.shape[1] != 2 or first.shape[0] < 1:
        raise ValueError('销载荷布局须为行星数×2')
    count = first.shape[0]
    pin_options = _per_planet(pins, count, '销参数')
    rating_options = _per_planet(bearing_ratings, count, '行星轴承评级参数')
    rating_fields = {'dynamic_capacity_n', 'static_capacity_n', 'exponent', 'radial_factor',
                     'axial_factor', 'static_radial_factor', 'static_axial_factor'}
    if any(set(options) != rating_fields for options in rating_options):
        raise ValueError('行星轴承须显式给出动/静容量、寿命指数及全部X/Y目录分支系数')
    if specified_pin_stiffness_n_per_m is None:
        specified = [None]*count
    else:
        raw = np.asarray(specified_pin_stiffness_n_per_m)
        if raw.dtype.kind not in 'iuf':
            raise ValueError('指定销刚度必须为正有限实数')
        array = np.asarray(raw, dtype=float)
        if array.ndim == 0:
            array = np.full(count, float(array))
        if array.shape != (count,) or not np.all(np.isfinite(array)) or np.any(array <= 0):
            raise ValueError('指定销刚度须为正有限标量或逐行星数组')
        specified = array.tolist()
    per_case = []
    for index, (case, segment) in enumerate(zip(cases, duty_segments)):
        forces = np.asarray(case['sharing']['carrier_pin_forces_xy_n'])
        if forces.shape != (count, 2) or forces.dtype.kind not in 'iuf' or not np.all(np.isfinite(forces)):
            raise ValueError('各工况销载荷必须保持行星数×2的有限实数布局')
        _scalar(segment['duration_s'], '工况时长s', positive=True)
        _scalar(case['speeds']['planet_relative_to_carrier_rad_s'], '行星相对行星架转速rad/s')
        per_case.append({'index': index, 'label': case.get('label', str(index+1)),
                         'pins': [pin_response(forces[j], **pin_options[j]) for j in range(count)]})
    summaries, bearings = [], []
    for j in range(count):
        states = [case['pins'][j] for case in per_case]
        life = bearing_duty_life([s['load_n'] for s in states], [0.0]*len(states),
                    [case['speeds']['planet_relative_to_carrier_rad_s'] for case in cases],
                    [segment['duration_s'] for segment in duty_segments], **rating_options[j])
        life.update(planet_index=j, speed_reference='planet_relative_to_carrier_rad_s',
                    axial_load_assumption='当前直齿NGW不产生轴向啮合力；轴承外圈转动等效系数由目录输入给定')
        bearings.append(life)
        summary = {'planet_index': j, 'maximum_load_n': max(s['load_n'] for s in states),
                   'maximum_bending_stress_pa': max(s['bending_stress_pa'] for s in states),
                   'maximum_transverse_shear_stress_pa': max(s['maximum_transverse_shear_stress_pa'] for s in states),
                   'maximum_projected_average_pressure_pa': max(s['projected_average_pressure_pa'] for s in states),
                   'maximum_center_deflection_m': max(s['center_deflection_m'] for s in states),
                   'maximum_load_weighted_deflection_m': max(s['load_weighted_deflection_m'] for s in states),
                   'equivalent_pin_stiffness_n_per_m': states[0]['equivalent_pin_stiffness_n_per_m'],
                   'specified_pin_stiffness_n_per_m': specified[j],
                   'stiffness_ratio_to_system_input': states[0]['equivalent_pin_stiffness_n_per_m']/specified[j] if specified[j] is not None else None,
                   'static_checks_pass': all(s['static_checks_pass'] for s in states),
                   'short_pin_review_required': states[0]['diagnostics']['short_pin_review_required']}
        for kind in ('bending', 'shear', 'projected_pressure'):
            factors = [s['checks'][kind]['safety_factor'] for s in states if s['checks'][kind]['safety_factor'] is not None]
            summary['minimum_'+kind+'_safety_factor'] = min(factors) if factors else None
        summaries.append(summary)
    return {'schema_version': 1, 'pin_inputs': pin_options, 'bearing_inputs': rating_options,
            'cases': per_case, 'pins': summaries, 'planet_bearings': bearings,
            'summary': {'all_pin_static_checks_pass': all(s['static_checks_pass'] for s in summaries),
                        'maximum_pin_load_n': max(s['maximum_load_n'] for s in summaries),
                        'minimum_basic_planet_bearing_life_hours': min((b['life_hours'] for b in bearings if b['life_hours'] is not None), default=None),
                        'minimum_planet_bearing_static_safety_factor': min((b['minimum_static_safety_factor'] for b in bearings if b['minimum_static_safety_factor'] is not None), default=None),
                        'system_stiffness_was_updated': False, 'physical_validation': False},
            'scope': '给定已收敛销反力下的独立静强度/线弹性挠度与基本L10初筛；未反馈重算分载，不构成整关节强度或摆动寿命认证'}
