"""关节计算原数组、中文设计报告与复算清单导出。"""
from dataclasses import asdict
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from gear_contact.io import json_value, export_result as export_mesh


def _write(path, value):
    Path(path).write_text(json.dumps(json_value(value), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def export_joint(result, output_dir, *, plots=False):
    """导出全部最终接触节点，不使用pickle，不以插值替代原计算数据。"""
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    _write(root/'input.json', result.inputs)
    _write(root/'summary.json', result.summary)
    _write(root/'motor.json', result.motor)
    _write(root/'thermal.json', result.thermal)
    _write(root/'bearings.json', result.bearings)
    _write(root/'fatigue.json', result.fatigue)
    # 无选项时明确写null，防止同一输出目录残留上次已配置的校核文件。
    _write(root/'planet_support.json', result.planet_support)
    _write(root/'dc_bus.json', result.dc_bus)
    from .engineering import export_engineering_parameters
    export_engineering_parameters(result, root)
    rows = []
    for case in result.cases:
        case_root = root/f"case_{case['index']+1:03d}"
        case_root.mkdir(exist_ok=True)
        for key in ('sharing', 'ratings', 'support', 'bearing_reactions', 'precision', 'speeds', 'kinematic_exposure'):
            _write(case_root/(key+'.json'), case[key])
        for name, group in case['contacts'].items():
            for index, mesh in enumerate(group):
                export_mesh(mesh, case_root/name/f'planet_{index+1:02d}')
        for index, force in enumerate(case['sharing']['tangential_forces_n']):
            rows.append([case['index']+1, case['label'], index+1, force,
                         case['sharing']['normal_forces_n'][index], case['sharing']['load_share_factor'],
                         case['precision']['output_torsional_stiffness_n_m_per_rad'],
                         case['precision']['deterministic_worst_case_error_rad']])
    with (root/'planets.csv').open('w', encoding='utf-8-sig', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case', 'label', 'planet', 'tangential_force_n', 'normal_force_n',
                         'load_share_factor', 'output_stiffness_n_m_per_rad', 'angle_budget_rad'])
        writer.writerows(rows)
    lines = ['# NGW关节模组设计计算报告', '',
             f"方案：{result.inputs['case_id']}。速比：{result.geometry['ratio']:.6g}。", '',
             '模型：单级固定内齿圈、零变位直齿NGW、小变形准静态与周期平均LTCA协调。', '',
             '## 关键结果', '',
             '|指标|结果|', '|---|---:|',
             f"|最大行星均载系数|{result.summary['maximum_load_share_factor']}|",
             f"|最大局部接触压力 / MPa|{result.summary['maximum_contact_pressure_pa']/1e6:.6g}|",
             f"|最小输出扭转刚度 / N·m/rad|{result.summary['minimum_output_torsional_stiffness_n_m_per_rad']:.6g}|",
             f"|最大确定性角度预算 / rad|{result.summary['maximum_angle_budget_rad']:.6g}|",
             f"|电机最高温度 / °C|{result.summary['maximum_motor_temperature_k']-273.15:.4f}|",
             f"|减速器最高温度 / °C|{result.summary['maximum_gear_temperature_k']-273.15:.4f}|", '',
             f"热计算模式：{result.summary['thermal_mode']}。", '',
             '## 约束初筛', '', '|项目|实际值|比较|限值|状态|', '|---|---:|---|---:|---|']
    statuses = {'passed': '通过', 'failed': '未通过', 'not_configured': '未配置限值',
                'not_evaluated': '缺少评价输入/未定义'}
    for name, detail in result.summary['check_details'].items():
        actual = '—' if detail['actual'] is None else f"{detail['actual']:.6g}"
        threshold = '—' if detail['threshold'] is None else f"{detail['threshold']:.6g}"
        lines.append(f"|{name}|{actual}|{detail['relation']}|{threshold}|{statuses[detail['status']]}|")
    lines.extend(['', '未配置限值、缺少评价输入及未建模项不计入通过项目。给定热点谱的默认Miner损伤限值为1，'
                  '只评价该输入谱，不等同整关节服役寿命。', '', '## 设计评估覆盖', '', '|项目|当前范围|', '|---|---|'])
    for name, scope in result.summary['assessment_coverage'].items():
        lines.append(f'|{name}|{scope}|')
    if result.planet_support is not None:
        lines.extend(['', '## 行星销与行星轴承', '',
            '按已收敛销反力独立检查，未自动回写路径刚度。短粗销三维接触、支承板、局部配合和销疲劳仍需单独验证。', '',
            '|行星|销平均弯剪刚度 / MN/m|与系统kpin之比|最大弯曲 / MPa|最大中心挠度 / μm|静校核|', '|---|---:|---:|---:|---:|---|'])
        for pin in result.planet_support['pins']:
            lines.append(f"|{pin['planet_index']+1}|{pin['equivalent_pin_stiffness_n_per_m']/1e6:.6g}|"
                         f"{pin['stiffness_ratio_to_system_input']:.6g}|{pin['maximum_bending_stress_pa']/1e6:.6g}|"
                         f"{pin['maximum_center_deflection_m']*1e6:.6g}|{'通过' if pin['static_checks_pass'] else '未通过'}|")
        lines.extend(['', '行星轴承基本L10使用相对行星架转速，不含摆动/润滑/微动修正；小时寿命仅在明确配置下限时参与判定。'])
    if result.dc_bus is not None:
        bus = result.dc_bus
        lines.extend(['', '## 直流母线能量初筛', '',
            f"最高母线电压{bus['maximum_voltage_v']:.6g} V，给定上限{bus['parameters']['maximum_voltage_v']:.6g} V。",
            f"当前历程回馈DC能量{bus['regenerated_dc_energy_j']:.6g} J，外部吸收{bus['externally_absorbed_energy_j']:.6g} J，"
            f"最大分段能量残差{bus['maximum_segment_energy_residual_j']:.3g} J。", '',
            '最低电压由理想无限供能源保证，不算真实欠压校核；单次输入历程通过不能替代重复任务能量平衡或真实驱动器认证。'])
    lines.extend(['', '## 逐工况载荷与精度', '',
                  '|工况|行星切向力 / N|均载系数|角度预算 / rad|协调残差 / m|', '|---|---|---:|---:|---:|'])
    for case in result.cases:
        forces = ' / '.join(f'{f:.4f}' for f in case['sharing']['tangential_forces_n'])
        lines.append(f"|{case['label']}|{forces}|{case['sharing']['load_share_factor']}|"
                     f"{case['precision']['deterministic_worst_case_error_rad']:.6g}|"
                     f"{case['sharing']['coupling']['path_residual_m']:.3g}|")
    lines.extend(['', '## 验证状态与限制', '',
                  '计算接口可独立复算；新写系统层已有解析及方程核验。厂商总成数值对标和实物试验尚未完成。', '',
                  '评级因子、材料限值、支承矩阵、摩擦效率、热参数与S-N数据来自输入；示例参数用于展示计算流程，未经具体产品标定。', '',
                  '角度预算为未补偿确定性上界；静态双工作面包络不是真实迟滞。轴承滚动寿命模型不含小角摆动、微动磨损和假压痕修正。', '',
                  '疲劳仅针对给定标量应力历程；未配置的失效模式不自动视为通过。', '',
                  '接触原数组见各case目录的arrays.npz；导出采用SI，图表单位另有标注。', ''])
    (root/'report.md').write_text('\n'.join(lines), encoding='utf-8')
    if plots:
        from .plotting import plot_joint
        plot_joint(result, root)
    manifest = {'version': '0.3.3', 'units': {'length': 'm', 'angle': 'rad', 'torque': 'N m',
                'force': 'N', 'pressure': 'Pa', 'temperature': 'K', 'time': 's'},
                'files': {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in sorted(root.rglob('*')) if p.is_file() and p.name != 'manifest.json'}}
    _write(root/'manifest.json', manifest)
    return root
