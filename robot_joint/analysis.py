"""NGW关节参数化设计主流程：载荷、支承、LTCA、评级、机电与热。"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import numpy as np

from .planetary import PlanetaryStage, stage_geometry, speed_map
from .gears import gear_pairs, coupled_planet_contacts, rate_mesh
from .support import support_response, two_bearing_reactions, bearing_duty_life
from .electromechanical import motor_duty
from .thermal import thermal_response
from .fatigue import rainflow_cycles, sn_damage
from .precision import precision_budget


def _keys(value, allowed, required, name):
    if not isinstance(value, dict) or set(value)-set(allowed) or set(required)-set(value):
        raise ValueError(name+'包含未知字段或缺少必要参数')


@dataclass
class JointResult:
    """完整计算结果；cases中的contacts保留原接触数组，导出时另存NPZ。"""
    inputs: dict
    geometry: dict
    cases: list
    motor: dict
    thermal: dict
    bearings: list
    fatigue: dict
    summary: dict


def analyze_joint(config: dict) -> JointResult:
    """计算一个单级直齿NGW关节方案；输入SI，原始配置不被修改。

    结构6自由度刚度表示输出接口串联结构，须排除齿面及已经凝聚的
    行星路径柔度。失准传递矩阵由调用方提供，不能从外部载荷推断其
    FE真实性。所有未计算的物理功能在summary.scope中明确列出。
    """
    if not isinstance(config, dict):
        raise ValueError('关节输入必须为对象')
    values = deepcopy(config)
    required = {'schema_version', 'case_id', 'stage', 'duty_segments', 'load_sharing',
                'mesh', 'support', 'bearings', 'motor', 'drive', 'thermal', 'precision'}
    allowed = required | {'label', 'rating', 'fatigue', 'design_limits', 'notes', 'thermal_mode'}
    if required-set(values) or set(values)-allowed:
        raise ValueError('关节输入缺少必要字段或含未知字段：' + str(sorted((required-set(values)) | (set(values)-allowed))))
    if isinstance(values['schema_version'], bool) or values['schema_version'] != 1:
        raise ValueError('仅支持schema_version=1')
    if not isinstance(values['case_id'], str) or not values['case_id']:
        raise ValueError('case_id必须为非空字符串')
    try:
        stage = PlanetaryStage(**values['stage'])
    except TypeError as error:
        raise ValueError('stage包含缺失或不支持的字段，首版仅支持零变位直齿单级NGW') from error
    geometry = stage_geometry(stage)
    pairs = gear_pairs(stage)
    _keys(values['load_sharing'], {'initial_mesh_stiffness_n_per_m', 'pin_stiffness_n_per_m',
          'sun_radial_stiffness_n_per_m', 'positive_path_gaps_m', 'negative_path_gaps_m',
          'extra_compliance_m_per_n'}, {'initial_mesh_stiffness_n_per_m', 'pin_stiffness_n_per_m',
          'sun_radial_stiffness_n_per_m'}, 'load_sharing')
    mesh = values['mesh']
    _keys(mesh, {'material', 'phases', 'strips', 'elastic_mode', 'modifications', 'alignment_maps',
          'sun_planet_misalignment_m', 'planet_ring_misalignment_m', 'coupling', 'iso_settings'},
          {'material', 'modifications'}, 'mesh')
    _keys(mesh.get('coupling', {}), {'maximum_iterations', 'force_relative_tolerance',
          'path_tolerance_m', 'relaxation'}, set(), 'mesh.coupling')
    _keys(mesh.get('alignment_maps', {}), {'sun_planet', 'planet_ring'}, set(), 'mesh.alignment_maps')
    _keys(mesh['modifications'], {'positive', 'negative'}, {'positive', 'negative'}, 'mesh.modifications')
    for direction in ('positive', 'negative'):
        _keys(mesh['modifications'][direction], set(pairs), set(pairs), '工作面修形')
        for pair_options in mesh['modifications'][direction].values():
            _keys(pair_options, {'pinion', 'wheel'}, set(), '单副修形')
    for pair in pairs.values():
        if pair.transverse_contact_ratio <= 1:
            raise ValueError('首版直齿模型要求两副端面重合度均大于1，以保证完整周期连续啮合')
    segments = values['duty_segments']
    if not isinstance(segments, list) or not segments:
        raise ValueError('duty_segments必须为非空列表')
    motor_segments = []
    for segment in segments:
        keys = {'label', 'duration_s', 'output_torque_n_m', 'output_speed_rad_s',
                'output_acceleration_rad_s2', 'output_wrench'}
        if not isinstance(segment, dict) or set(segment)-keys:
            raise ValueError('工况必须为已定义字段的对象')
        if 'output_wrench' not in segment:
            raise ValueError('每个工况必须显式给定输出端6分量output_wrench')
        wrench = np.asarray(segment['output_wrench'], dtype=float)
        if wrench.shape != (6,) or not np.all(np.isfinite(wrench)):
            raise ValueError('output_wrench必须为6分量有限数[N,N,N,Nm,Nm,Nm]')
        if 'output_torque_n_m' not in segment or not math.isclose(float(wrench[5]), segment['output_torque_n_m'], rel_tol=1e-9, abs_tol=1e-10):
            raise ValueError('output_wrench的Mz必须与output_torque_n_m采用相同载荷需求值和符号')
        motor_segments.append({key: val for key, val in segment.items()
                               if key in {'duration_s', 'output_torque_n_m', 'output_speed_rad_s',
                                          'output_acceleration_rad_s2'}})
    motor = motor_duty(motor_segments, geometry['ratio'], motor=values['motor'], **values['drive'])
    support_options = values['support']
    if 'stiffness_matrix' not in support_options or set(support_options)-{'stiffness_matrix', 'reference_offset_m'}:
        raise ValueError('support须给定stiffness_matrix，可附reference_offset_m；结构边界必须显式给出')
    bearing_options = values['bearings']
    if set(bearing_options)-{'spacing_m', 'load_offset_m', 'axial_share', 'ratings'}:
        raise ValueError('bearings含未知字段')
    if len(bearing_options['ratings']) != 2:
        raise ValueError('首版双支承接口必须给定两组轴承评级参数')
    cases = []
    for index, segment in enumerate(segments):
        support = support_response(segment['output_wrench'], support_options['stiffness_matrix'],
                                   reference_offset_m=support_options.get('reference_offset_m', 0.0))
        displacement = np.asarray(support['generalized_displacement'])
        bearing = two_bearing_reactions(segment['output_wrench'],
                    bearing_options['spacing_m'], bearing_options.get('load_offset_m', 0),
                    bearing_options.get('axial_share', 0.5))
        sharing, contacts = coupled_planet_contacts(stage, segment['output_torque_n_m'],
                    values['load_sharing'], values['mesh'], support_displacement=displacement)
        ratings = {}
        if 'rating' in values:
            if set(values['rating']) != set(pairs):
                raise ValueError('rating必须分别给出sun_planet和planet_ring的显式因子与材料限值')
            for name, pair in pairs.items():
                ratings[name] = [rate_mesh(pair, force, values['mesh']['material'], values['rating'][name])
                                 for force in sharing['tangential_forces_n']]
        gear_k = sharing['output_torsional_stiffness_n_m_per_rad']
        compliance = np.asarray(support['load_point_compliance'])
        total_k = 1 / (1/gear_k + compliance[5, 5])
        precision = precision_budget(
            output_elastic_twist_rad=sharing['output_elastic_twist_rad']+float(displacement[5]),
            output_torsional_stiffness_n_m_per_rad=float(total_k), ratio=geometry['ratio'],
            **values['precision'])
        speeds = speed_map(stage, segment['output_speed_rad_s'])
        duration = segment['duration_s']
        exposure = {'sun_tooth_mesh_passes': abs(speeds['sun_rad_s']-speeds['carrier_rad_s'])*duration/(2*math.pi)*stage.planet_count,
                    'planet_tooth_passes_per_mesh': abs(speeds['planet_relative_to_carrier_rad_s'])*duration/(2*math.pi),
                    'ring_tooth_mesh_passes': abs(speeds['carrier_rad_s'])*duration/(2*math.pi)*stage.planet_count,
                    'scope': '运动学受载机会数；不直接等于疲劳循环数，不含摆动接触覆盖修正'}
        cases.append({'index': index, 'label': segment.get('label', str(index+1)),
                      'sharing': sharing, 'contacts': contacts, 'ratings': ratings,
                      'support': support, 'bearing_reactions': bearing, 'precision': precision,
                      'speeds': speeds, 'kinematic_exposure': exposure})
    bearings = []
    for index, rating in enumerate(bearing_options['ratings']):
        bearings.append(bearing_duty_life(
            [c['bearing_reactions']['radial_loads_n'][index] for c in cases],
            [c['bearing_reactions']['axial_loads_n'][index] for c in cases],
            [s['output_speed_rad_s'] for s in segments], [s['duration_s'] for s in segments], **rating))
    thermal_segments = [{name: segment[name] for name in ('duration_s', 'motor_loss_w', 'gear_loss_w')}
                        for segment in motor['segments']]
    thermal_mode = values.get('thermal_mode', 'single_duty')
    if thermal_mode == 'single_duty':
        thermal = thermal_response(thermal_segments, **values['thermal'])
    elif thermal_mode == 'periodic_steady':
        from .thermal import periodic_thermal_response
        thermal = periodic_thermal_response(thermal_segments, **values['thermal'])
    else:
        raise ValueError('thermal_mode必须为single_duty或periodic_steady')
    fatigue = {'status': '未配置标量应力历程与S-N参数', 'physical_validation': False}
    if 'fatigue' in values:
        options = dict(values['fatigue'])
        stresses = options.pop('stress_history_pa')
        cycles = rainflow_cycles(stresses)
        fatigue = sn_damage(cycles, **options)
        fatigue['cycles'] = cycles
        fatigue['scope'] = '调用方给定热点应力历程上的有限寿命模型；不代表所有齿轮、轴承和结构的寿命'
    peak_pressure = max(a.summary['max_pressure_pa'] for c in cases for group in c['contacts'].values() for a in group)
    factors = [c['sharing']['load_share_factor'] for c in cases if c['sharing']['load_share_factor'] is not None]
    checks, check_details = {}, {}

    def record_check(name, actual, threshold, relation, *, scope):
        """同时保存实际值、限值与比较方向，不把未配置约束算作通过。"""
        passed = actual <= threshold if relation == '<=' else actual >= threshold
        checks[name] = bool(passed)
        check_details[name] = {'status': 'passed' if passed else 'failed',
                               'actual': float(actual), 'threshold': float(threshold),
                               'relation': relation, 'scope': scope}

    record_check('motor_speed', max(abs(s['motor_speed_rad_s']) for s in motor['segments']),
                 values['motor']['maximum_speed_rad_s'], '<=', scope='电机代表工况最大转速，rad/s')
    record_check('motor_peak_torque', max(abs(s['motor_torque_n_m'])/s['peak_torque_limit_n_m']
                 for s in motor['segments']), 1.0, '<=', scope='逐工况恒转矩—恒功率峰值包络利用率')
    record_check('motor_continuous_screen', motor['normalized_continuous_torque_rms'], 1.0,
                 '<=', scope='按转速包络归一化的连续转矩RMS初筛，不替代温升校核')
    record_check('brake_static_screen', motor['brake_available_torque_n_m'],
                 motor['brake_required_torque_n_m'], '>=', scope='输入侧静态保持力矩，N·m；不含动态制动认证')
    gear_ratings = [rating[role] for case in cases for group in case['ratings'].values()
                    for rating in group for role in ('pinion', 'wheel')]
    contact_safety = [r['contact_safety_factor'] for r in gear_ratings if r['contact_safety_factor'] is not None]
    bending_safety = [r['bending_safety_factor'] for r in gear_ratings if r['bending_safety_factor'] is not None]
    static_safety = [b['minimum_static_safety_factor'] for b in bearings if b['minimum_static_safety_factor'] is not None]
    bearing_lives = [b['life_hours'] for b in bearings if b['life_hours'] is not None]
    summary = {'case_id': values['case_id'], 'ratio': geometry['ratio'],
               'maximum_load_share_factor': max(factors) if factors else None,
               'maximum_contact_pressure_pa': peak_pressure,
               'minimum_output_torsional_stiffness_n_m_per_rad': min(c['precision']['output_torsional_stiffness_n_m_per_rad'] for c in cases),
               'maximum_angle_budget_rad': max(c['precision']['deterministic_worst_case_error_rad'] for c in cases),
               'maximum_motor_temperature_k': thermal['maximum_temperature_k'][0],
               'maximum_gear_temperature_k': thermal['maximum_temperature_k'][1],
               'thermal_mode': thermal_mode,
               'minimum_contact_safety_factor': min(contact_safety) if contact_safety else None,
               'minimum_bending_safety_factor': min(bending_safety) if bending_safety else None,
               'minimum_bearing_static_safety_factor': min(static_safety) if static_safety else None,
               'minimum_bearing_life_hours': min(bearing_lives) if bearing_lives else None,
               'fatigue_damage_per_input_history': fatigue.get('damage_per_duty'),
               'all_couplings_converged': all(c['sharing']['coupling']['converged'] for c in cases),
               'checks': checks, 'check_details': check_details,
               'physical_validation': False, 'vendor_system_validation': False,
               'implementation': '新写NGW降阶系统实现，复用公开齿轮内核；非厂商整机等价重建',
               'scope': '固定环零变位直齿单级NGW、准静态、小变形、周期平均协调；不含RV/谐波、完整FE、同步动力学、控制、润滑预测或实物认证'}
    limits = values.get('design_limits', {})
    metric_names = {'contact_pressure_pa': 'maximum_contact_pressure_pa',
                    'angle_budget_rad': 'maximum_angle_budget_rad',
                    'motor_temperature_k': 'maximum_motor_temperature_k',
                    'gear_temperature_k': 'maximum_gear_temperature_k',
                    'maximum_load_share_factor': 'maximum_load_share_factor',
                    'fatigue_damage_per_input_history': 'fatigue_damage_per_input_history'}
    lower_metrics = {'minimum_contact_safety_factor': 'minimum_contact_safety_factor',
                     'minimum_bending_safety_factor': 'minimum_bending_safety_factor',
                     'minimum_bearing_static_safety_factor': 'minimum_bearing_static_safety_factor',
                     'minimum_bearing_life_hours': 'minimum_bearing_life_hours',
                     'minimum_output_torsional_stiffness_n_m_per_rad':
                         'minimum_output_torsional_stiffness_n_m_per_rad'}
    scopes = {'contact_pressure_pa': '当前离散局部Hertz峰压，Pa；须另查网格/相位收敛',
              'angle_budget_rad': '指定受载状态的未补偿确定性角度预算，rad',
              'motor_temperature_k': '给定热参数的电机节点最高温度，K',
              'gear_temperature_k': '给定热参数的传动节点最高温度，K',
              'maximum_load_share_factor': '有载工况最大均载系数；全零载时未定义',
              'fatigue_damage_per_input_history': '给定热点标量应力谱的Miner损伤；不代表整关节服役寿命',
              'minimum_contact_safety_factor': '给定评级因子与材料限值的最小接触安全系数',
              'minimum_bending_safety_factor': '给定评级因子与材料限值的最小齿根安全系数',
              'minimum_bearing_static_safety_factor': '输出双支承基本静安全；不含行星轴承',
              'minimum_bearing_life_hours': '输出双支承基本L10小时数；无摆动/润滑/微动修正',
              'minimum_output_torsional_stiffness_n_m_per_rad': '最终承载分支局部输出扭转刚度，N·m/rad'}
    for name, metric in {**metric_names, **lower_metrics}.items():
        actual = summary[metric]
        check_details[name] = {'status': 'not_configured' if actual is not None else 'not_evaluated',
                               'actual': actual, 'threshold': None,
                               'relation': '<=' if name in metric_names else '>=',
                               'scope': scopes[name]}
    if set(limits)-set(metric_names)-set(lower_metrics):
        raise ValueError('design_limits含不支持的指标')
    for name, threshold in limits.items():
        if isinstance(threshold, bool) or not np.isfinite(threshold) or threshold <= 0:
            raise ValueError('方案限值必须为正有限值')
        if name in metric_names:
            value = summary[metric_names[name]]
            relation = '<='
        else:
            value = summary[lower_metrics[name]]
            relation = '>='
        if value is None:
            raise ValueError(name+'没有已计算的有限评价量，不能校核此限值')
        record_check(name, value, threshold, relation, scope=scopes[name])
    for name in ('minimum_contact_safety_factor', 'minimum_bending_safety_factor', 'minimum_bearing_static_safety_factor'):
        if summary[name] is not None and name not in limits:
            record_check(name, summary[name], 1.0, '>=', scope=scopes[name])
    if summary['fatigue_damage_per_input_history'] is not None and 'fatigue_damage_per_input_history' not in limits:
        # 只评价本次明确给定的热点谱，不能推广成齿轮全寿命或整关节认证。
        record_check('fatigue_damage_per_input_history', summary['fatigue_damage_per_input_history'],
                     1.0, '<=', scope=scopes['fatigue_damage_per_input_history'])
    summary['assessment_coverage'] = {
        'output_bearing_life': '仅基本L10；小角摆动、微动和润滑修正未评估',
        'hotspot_fatigue': '仅给定标量应力谱' if 'fatigue' in values else '未配置应力谱与S-N数据',
        'planet_pin_strength': '未评估；等效销刚度不等同销强度',
        'planet_bearing_life': '未评估；当前轴承仅为两个输出支承',
        'ring_carrier_housing_strength': '未评估；未构造完整实体和FE边界',
        'regenerative_bus_and_inverter': '未评估；机电功率平衡不等同母线吸能和驱动器能力',
        'dynamics_and_resonance': '未评估；当前为周期平均准静态协调',
        'manufacturing_interference': '仅外齿轮标准全齿高齿条根切风险初筛；内齿干涉、尖顶和刀具未评估',
        'physical_validation': '未完成样机标定与试验'}
    summary['configured_checks_pass'] = all(checks.values())
    summary['check_scope'] = '仅对已配置约束的数值初筛；未配置强度/寿命/温升限值不自动视为已校核'
    return JointResult(values, geometry, cases, motor, thermal, bearings, fatigue, summary)


def compare_joint_designs(configs):
    """对显式给定的完整候选重算，返回透明的约束与非支配比较。

    外部评级因子必须随几何候选重新提供，不能固定一套因子冒称自动评级。
    """
    if not isinstance(configs, list) or not configs:
        raise ValueError('候选必须为非空配置列表')
    results = [analyze_joint(config) for config in configs]
    metrics = ('maximum_contact_pressure_pa', 'maximum_angle_budget_rad', 'maximum_motor_temperature_k')
    rows = []
    for i, result in enumerate(results):
        dominated = any(other.summary['configured_checks_pass'] and all(other.summary[k] <= result.summary[k] for k in metrics)
                        and any(other.summary[k] < result.summary[k] for k in metrics)
                        for j, other in enumerate(results) if j != i)
        rows.append({'case_id': result.summary['case_id'], 'configured_checks_pass': result.summary['configured_checks_pass'],
                     'pareto_nondominated': result.summary['configured_checks_pass'] and not dominated,
                     **{k: result.summary[k] for k in metrics}})
    return {'metrics_minimized': list(metrics), 'candidates': rows, 'results': results}
