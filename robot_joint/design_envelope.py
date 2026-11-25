"""新写的装配/支承情景复算与约束裕量，不把有限情景解释为统计可靠度。"""
from copy import deepcopy
import math
import re

from .analysis import analyze_joint
from .gears import CouplingConvergenceError


def constraint_margins(result):
    """从已计算判据取有符号裕量，定位可追溯的首个危险工况。

    上限裕量=限值−实际，下限裕量=实际−限值；相对裕量除以|限值|。
    未评价、未设限值或零限值不构造百分比。累计任务指标不伪造单一工况。
    """
    candidates = {}

    def add(name, value, **where):
        if value is not None and math.isfinite(value):
            candidates.setdefault(name, []).append((float(value), where))

    for index, case in enumerate(result.cases):
        where = {'case_index': index, 'label': case['label']}
        add('maximum_load_share_factor', case['sharing']['load_share_factor'], **where)
        add('minimum_output_torsional_stiffness_n_m_per_rad',
            case['precision']['output_torsional_stiffness_n_m_per_rad'], **where)
        add('angle_budget_rad', case['precision']['deterministic_worst_case_error_rad'], **where)
        motor = result.motor['segments'][index]
        add('motor_speed', abs(motor['motor_speed_rad_s']), **where)
        add('motor_peak_torque', abs(motor['motor_torque_n_m'])/motor['peak_torque_limit_n_m'], **where)
        for mesh, group in case['contacts'].items():
            for planet, contact in enumerate(group):
                add('contact_pressure_pa', contact.summary['max_pressure_pa'], mesh=mesh,
                    planet_index=planet, **where)
        for mesh, group in case['ratings'].items():
            for planet, rating in enumerate(group):
                for role in ('pinion', 'wheel'):
                    values = rating[role]
                    for kind in ('contact', 'bending'):
                        add('minimum_'+kind+'_safety_factor', values[kind+'_safety_factor'],
                            mesh=mesh, planet_index=planet, gear_role=role, **where)
    support = result.planet_support
    if support:
        for case in support['cases']:
            for planet, pin in enumerate(case['pins']):
                for kind, check in pin['checks'].items():
                    add('planet_pin_static', check['actual_pa']*check['required_safety_factor']/check['allowable_pa'],
                        case_index=case['index'], label=case['label'], planet_index=planet, stress_kind=kind)
        for planet, pin in enumerate(support['pins']):
            ratio = pin['stiffness_ratio_to_system_input']
            add('maximum_pin_stiffness_relative_difference', abs(ratio-1) if ratio is not None else None,
                planet_index=planet)
    for name, bearings, ratings, index_name in (
        ('minimum_bearing_static_safety_factor', result.bearings, result.inputs['bearings']['ratings'], 'bearing_index'),
        ('minimum_planet_bearing_static_safety_factor', support['planet_bearings'] if support else [],
         support['bearing_inputs'] if support else [], 'planet_index')):
        for j, (bearing, rating) in enumerate(zip(bearings, ratings)):
            for i, load in enumerate(bearing['static_equivalent_load_n']):
                add(name, rating['static_capacity_n']/load if load > 0 else None,
                    case_index=i, label=result.cases[i]['label'], **{index_name: j})
    rows = {}
    for name, detail in result.summary['check_details'].items():
        row = deepcopy(detail)
        row.update(margin=None, relative_margin=None, governing_location=None)
        actual, threshold = row['actual'], row['threshold']
        if row['status'] in ('passed', 'failed'):
            if (actual is None or threshold is None or not math.isfinite(actual)
                    or not math.isfinite(threshold) or row['relation'] not in ('<=', '>=')):
                raise ValueError('已评价约束缺少有效数值或比较方向：'+name)
            row['margin'] = threshold-actual if row['relation'] == '<=' else actual-threshold
            row['relative_margin'] = row['margin']/abs(threshold) if threshold != 0 else None
        if candidates.get(name) and actual is not None:
            choose = max if row['relation'] == '<=' else min
            value, where = choose(candidates[name], key=lambda item: item[0])
            # 只有局部极值能重现实际汇总值时才登记，避免错误指认累计指标。
            if math.isclose(value, actual, rel_tol=1e-10, abs_tol=1e-12):
                row['governing_location'] = where
        rows[name] = row
    return rows


def _scenario_config(base, scenario):
    """只允许装配/支承参数变化，保持几何评级、任务和验收限值一致。"""
    if not isinstance(scenario, dict) or set(scenario)-{'id', 'description', 'overrides'}:
        raise ValueError('情景只接受id、description和overrides')
    identifier = scenario.get('id')
    if not isinstance(identifier, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', identifier) or identifier == 'nominal':
        raise ValueError('情景id须为唯一字母/数字/下划线/连字符，不能使用nominal')
    if not isinstance(scenario.get('description', ''), str):
        raise ValueError('情景description必须为文字')
    changes = scenario.get('overrides')
    allowed = {'load_sharing': {'positive_path_gaps_m', 'negative_path_gaps_m',
        'sun_radial_stiffness_n_per_m', 'pin_stiffness_n_per_m', 'extra_compliance_m_per_n'},
        'support': {'stiffness_matrix'},
        'mesh': {'sun_planet_misalignment_m', 'planet_ring_misalignment_m'},
        'planet_support': {'pins'}}
    if not isinstance(changes, dict) or not changes or set(changes)-set(allowed):
        raise ValueError('情景overrides须为非空的装配/支承参数对象')
    config = deepcopy(base)
    for section, fields in changes.items():
        if not isinstance(fields, dict) or not fields or set(fields)-allowed[section]:
            raise ValueError('情景存在不允许变化的字段：'+section)
        if section not in config:
            raise ValueError('基准没有配置情景所需模块：'+section)
        if section == 'planet_support':
            if base['load_sharing'].get('pin_stiffness_source', 'specified') != 'pin_geometry':
                raise ValueError('销参数变化须在基准中显式选择pin_geometry，使改变参与系统分载')
            count = base['stage']['planet_count']
            original = config[section]['pins']
            original = [deepcopy(original) for _ in range(count)] if isinstance(original, dict) else original
            patches = fields['pins']
            patches = [patches]*count if isinstance(patches, dict) else patches
            if not isinstance(patches, list) or len(patches) != count:
                raise ValueError('情景pins须为共享参数对象或逐行星参数列表')
            pin_fields = {'diameter_m', 'support_span_m', 'loaded_width_m',
                          'young_modulus_pa', 'poisson_ratio', 'shear_coefficient'}
            for j, patch in enumerate(patches):
                if not isinstance(patch, dict) or set(patch)-pin_fields:
                    raise ValueError('销情景只能改变几何、材料弹性参数与剪切系数，不能改变许用值')
                original[j].update(deepcopy(patch))
            config[section]['pins'] = original
        else:
            if (section == 'load_sharing' and 'pin_stiffness_n_per_m' in fields
                    and base['load_sharing'].get('pin_stiffness_source') == 'pin_geometry'):
                raise ValueError('pin_geometry情景不能改变未采用的显式刚度；请改变销参数')
            config[section].update(deepcopy(fields))
    config['case_id'] = base['case_id']+'__'+identifier
    return config


def analyze_joint_envelope(config, scenarios, *, required_checks):
    """对显式有限情景逐个重算完整任务，汇总最差约束与危险位置。

    自动包含nominal。required_checks为必须完成的已有判据名；未配置或
    未评价不能作为通过。情景不得修改任务、齿轮评级和验收限值。未收敛
    直接报错，返回值不带半成品。结果不是连续公差域包络或失效概率。
    """
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError('至少提供一个明确的装配/支承变化情景')
    if (not isinstance(required_checks, list) or not required_checks
            or not all(isinstance(v, str) and v for v in required_checks)
            or len(set(required_checks)) != len(required_checks)):
        raise ValueError('required_checks须为非空且不重复的判据名称列表')
    variants = [deepcopy(config)]+[_scenario_config(config, scenario) for scenario in scenarios]
    identifiers = ['nominal']+[s['id'] for s in scenarios]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError('情景id不能重复')
    results, records = [], []
    for identifier, candidate in zip(identifiers, variants):
        try:
            result = analyze_joint(candidate)
        except CouplingConvergenceError as error:
            raise CouplingConvergenceError('情景'+identifier+'未完成协调：'+str(error)) from error
        except ValueError as error:
            raise ValueError('情景'+identifier+'输入或计算失败：'+str(error)) from error
        margins = constraint_margins(result)
        unknown = set(required_checks)-set(margins)
        if unknown:
            raise ValueError('required_checks含未知判据：'+', '.join(sorted(unknown)))
        blocked = [name for name in required_checks if margins[name]['status'] not in ('passed', 'failed')]
        failed = [name for name, item in margins.items() if item['status'] == 'failed']
        records.append({'id': identifier, 'case_id': candidate['case_id'], 'checks': margins,
            'required_checks_missing': blocked, 'failed_checks': failed,
            'accepted': not blocked and not failed and result.summary['all_couplings_converged'],
            'summary': deepcopy(result.summary), 'pin_stiffness': deepcopy(result.pin_stiffness)})
        results.append(result)
    worst = {}
    for name in records[0]['checks']:
        evaluated = [(record['checks'][name], record['id']) for record in records
                     if record['checks'][name]['margin'] is not None]
        blocked = [record['id'] for record in records
                   if record['checks'][name]['status'] not in ('passed', 'failed')]
        if evaluated:
            row, identifier = min(evaluated, key=lambda item: item[0]['margin'])
            worst[name] = {**deepcopy(row), 'scenario_id': identifier,
                'unevaluated_scenarios': blocked}
        else:
            worst[name] = {**deepcopy(records[0]['checks'][name]), 'scenario_id': None,
                'unevaluated_scenarios': blocked}
    return {'schema_version': 1, 'base_input': deepcopy(config), 'scenario_definitions': deepcopy(scenarios),
        'required_checks': list(required_checks), 'scenarios': records, 'worst_checks': worst,
        'all_scenarios_accepted': all(row['accepted'] for row in records),
        'evaluated_scenario_count': len(records), 'results': results, 'physical_validation': False,
        'scope': '仅覆盖明确列出的有限确定性情景，不等于连续公差域最坏情况、统计可靠度或总成实物验证'}
