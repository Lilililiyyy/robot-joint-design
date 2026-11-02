"""用关节电机历程比较母线电容与外部回馈吸收能力；参数均为新设计假设。"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path

from robot_joint.dc_bus import dc_bus_screen
from robot_joint.electromechanical import motor_duty


def project_dir():
    """按独立项目标记定位，不依赖启动目录或固定父目录层数。"""
    return next(p for p in Path(__file__).resolve().parents
                if (p/'pyproject.toml').is_file() and (p/'robot_joint').is_dir())


def run_study(motor):
    """比较三种明确的设计假设，保持电机电功率历程不变。"""
    common = {'source_voltage_v': 36.0, 'minimum_voltage_v': 30.0,
              'maximum_voltage_v': 48.0, 'inverter_motoring_efficiency': .96,
              'inverter_regenerating_efficiency': .96}
    definitions = [('small_capacitor_no_sink', .000470, 0.0),
                   ('larger_capacitor_no_sink', .004700, 0.0),
                   ('small_capacitor_external_sink', .000470, 15.0)]
    cases = []
    for name, capacitance, absorption in definitions:
        result = dc_bus_screen(motor['segments'], **common, capacitance_f=capacitance,
                               absorption_power_limit_w=absorption)
        cases.append({'candidate': name, **result})
    return {'schema_version': 1, 'cases': cases,
            'assumptions_zh': '36V理想源、30/48V下上限、96%双向逆变效率、470/4700µF电容及0/15W外部回馈吸收能力均为新增设计假设，不是DFKI或原驱动器规格。',
            'assumptions_en': 'The ideal 36V source, 30/48V limits, 96% conversion efficiencies, 470/4700µF capacitors and 0/15W external absorption capacities are new design assumptions, not DFKI or original-drive specifications.',
            'physical_validation': False, 'driver_hardware_validation': False}


def main():
    root = project_dir()
    parser = argparse.ArgumentParser(description='关节历程的直流母线能量与过压初筛')
    parser.add_argument('--results', type=Path, help='可选的export_joint输出目录，读取motor.json')
    parser.add_argument('--input', type=Path, default=root/'examples/dfki_pendulum_case.json',
                        help='未提供--results时，仅由该配置重新计算机电历程，不重跑LTCA')
    parser.add_argument('--output', type=Path, default=root/'examples/results/dc_bus')
    args = parser.parse_args()
    if args.results:
        source = args.results.resolve()
        target = args.output.resolve()
        if target == source or source in target.parents:
            parser.error('输出目录不能位于原始导出证据目录内部')
        motor_bytes = (source/'motor.json').read_bytes()
        motor = json.loads(motor_bytes.decode('utf-8-sig'))
        provenance = {'kind': 'exported_motor_history', 'file': 'motor.json',
                      'sha256': hashlib.sha256(motor_bytes).hexdigest()}
    else:
        source_bytes = args.input.read_bytes()
        config = json.loads(source_bytes.decode('utf-8-sig'))
        keys = {'duration_s', 'output_torque_n_m', 'output_speed_rad_s', 'output_acceleration_rad_s2'}
        segments = [{key: value for key, value in segment.items() if key in keys}
                    for segment in config['duty_segments']]
        ratio = 1+config['stage']['ring_teeth']/config['stage']['sun_teeth']
        motor = motor_duty(segments, ratio, motor=config['motor'], **config['drive'])
        provenance = {'kind': 'recomputed_motor_history', 'input_case_id': config['case_id'],
                      'input_sha256': hashlib.sha256(source_bytes).hexdigest()}
    study = run_study(motor)
    study['source'] = provenance
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'study.json').write_text(json.dumps(study, ensure_ascii=False, indent=2,
                                                   allow_nan=False)+'\n', encoding='utf-8', newline='\n')
    summary_fields = ['candidate', 'maximum_voltage_v', 'maximum_voltage_time_s',
                      'maximum_regenerated_dc_power_w', 'regenerated_dc_energy_j',
                      'externally_absorbed_energy_j', 'capacitor_reused_for_motoring_energy_j',
                      'average_external_absorbed_power_w', 'maximum_external_absorbed_power_w',
                      'additional_peak_energy_buffer_required_j',
                      'minimum_capacitance_f_for_nominal_start', 'within_maximum_voltage',
                      'total_energy_balance_residual_j']
    with (args.output/'comparison.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_fields)
        writer.writeheader()
        writer.writerows({key: case[key] for key in summary_fields} for case in study['cases'])
    trace_rows = []
    for case in study['cases']:
        for time, value in zip(case['time_s'], case['voltage_v']):
            trace_rows.append({'candidate': case['candidate'], 'time_s': time, 'voltage_v': value})
    with (args.output/'voltage.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=['candidate', 'time_s', 'voltage_v'])
        writer.writeheader()
        writer.writerows(trace_rows)
    segment_rows = [{'candidate': case['candidate'], **segment}
                    for case in study['cases'] for segment in case['segments']]
    with (args.output/'segments.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(segment_rows[0]))
        writer.writeheader()
        writer.writerows(segment_rows)
    print(json.dumps({'candidates': [{key: case[key] for key in summary_fields}
                                    for case in study['cases']]}, ensure_ascii=False))


if __name__ == '__main__':
    main()
