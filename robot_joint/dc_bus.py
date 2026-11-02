"""新写的分段直流母线能量初筛；功率W、能量J、电压V、电容F。

采用恒功率代表工况、恒定逆变效率、理想供电电压下限和给定回馈吸收
上限。它不模拟PWM、线路阻抗、反电势、制动斩波器或驱动器故障动作。
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from numbers import Real


def _number(value, name, *, positive=False, nonnegative=False):
    """拒绝布尔、非数值和非有限输入，避免隐式单位与参数转换。"""
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(name+'必须为有限实数')
    result = float(value)
    if positive and result <= 0:
        raise ValueError(name+'必须大于0')
    if nonnegative and result < 0:
        raise ValueError(name+'不能为负')
    return result


def dc_bus_screen(segments, *, capacitance_f, source_voltage_v, minimum_voltage_v,
                  maximum_voltage_v, inverter_motoring_efficiency,
                  inverter_regenerating_efficiency, absorption_power_limit_w=0.0,
                  initial_voltage_v=None):
    """由电机侧有符号电功率，解析推进母线电容的能量和阈值事件。

    每段提供duration_s>0与electrical_power_w；正值为电机侧消耗，负值
    为已扣除电机损耗后的回馈。该量应直接取motor_duty结果，不能以负
    机械功率替代，否则会重复忽略铜损及转子惯性。

    驱动时Pdc=Pe/ηm，回馈时Pdc=Pe·ηr，逆变损耗Pdc-Pe>=0。
    U=C(V²-Vsource²)/2为高于源电压的可用电容能量。驱动先消耗U，
    U=0后由理想无限供能电源保持Vsource；回馈时外部吸收器最多吸收A，
    Udot=-Pdc-A；若U到0，吸收降到当时回馈功率，不能抽至源电压以下。
    此处A仅表示给定外部吸收能力，不是制动电阻或电池充电控制模型。

    段内U线性变化，到源电压后保持为0；电压由平方根精确恢复。保留
    上限跨越及回落到源电压的实际事件，不用时间步长掩盖过压。
    """
    capacitance = _number(capacitance_f, 'capacitance_f', positive=True)
    source = _number(source_voltage_v, 'source_voltage_v', positive=True)
    lower = _number(minimum_voltage_v, 'minimum_voltage_v', positive=True)
    upper = _number(maximum_voltage_v, 'maximum_voltage_v', positive=True)
    initial = source if initial_voltage_v is None else _number(initial_voltage_v, 'initial_voltage_v', positive=True)
    absorption = _number(absorption_power_limit_w, 'absorption_power_limit_w', nonnegative=True)
    eta_m = _number(inverter_motoring_efficiency, 'inverter_motoring_efficiency', positive=True)
    eta_r = _number(inverter_regenerating_efficiency, 'inverter_regenerating_efficiency', positive=True)
    if eta_m > 1 or eta_r > 1:
        raise ValueError('逆变效率必须位于(0,1]')
    if not lower <= source <= initial <= upper or upper <= source:
        raise ValueError('母线须满足0<最低电压≤源电压≤初始电压≤最高电压，且最高电压大于源电压')
    if not isinstance(segments, Sequence) or isinstance(segments, (str, bytes)) or not segments:
        raise ValueError('segments必须为非空工况序列')

    surplus = capacitance*(initial-source)*(initial+source)/2
    available = capacitance*(upper-source)*(upper+source)/2
    if not math.isfinite(surplus) or not math.isfinite(available):
        raise ValueError('电压和电容的组合导致能量超出有限数值范围')
    reference_surplus, reference_peak = 0.0, 0.0
    initial_surplus = surplus
    elapsed, peak_surplus, peak_time = 0.0, surplus, 0.0
    rows, events = [], []
    time_trace, voltage_trace = [0.0], [initial]

    def voltage(energy):
        result = math.sqrt(source*source+2*energy/capacitance)
        if not math.isfinite(result):
            raise ValueError('工况能量导致母线电压超出有限数值范围')
        return result

    for index, segment in enumerate(segments):
        if not isinstance(segment, Mapping) or not {'duration_s', 'electrical_power_w'} <= set(segment):
            raise ValueError('每段必须包含duration_s和electrical_power_w')
        duration = _number(segment['duration_s'], 'duration_s', positive=True)
        motor_power = _number(segment['electrical_power_w'], 'electrical_power_w')
        dc_power = motor_power/eta_m if motor_power >= 0 else motor_power*eta_r
        inverter_loss = dc_power-motor_power
        if not math.isfinite(dc_power) or not math.isfinite(dc_power*duration):
            raise ValueError('功率和时长的组合导致能量超出有限数值范围')
        begin_surplus = surplus
        begin_voltage = voltage(surplus)
        regeneration = max(-dc_power, 0.0)
        slope = -dc_power-(absorption if dc_power < 0 else 0.0)
        trial_surplus = surplus+slope*duration
        if not math.isfinite(trial_surplus):
            raise ValueError('分段能量推进超出有限数值范围')
        surplus = max(trial_surplus, 0.0)
        reference_surplus = max(reference_surplus+slope*duration, 0.0)
        reference_peak = max(reference_peak, reference_surplus)
        supply_energy = max(dc_power*duration-begin_surplus, 0.0) if dc_power >= 0 else 0.0
        absorbed_energy = regeneration*duration+begin_surplus-surplus if dc_power < 0 else 0.0
        reused_energy = min(begin_surplus, dc_power*duration) if dc_power >= 0 else 0.0
        residual = surplus-begin_surplus+dc_power*duration+absorbed_energy-supply_energy
        internal = []
        if slope < 0 and begin_surplus > 0 and trial_surplus <= 0:
            crossing = begin_surplus/(-slope)
            events.append({'segment_index': index, 'event': 'source_voltage_reached',
                           'time_s': elapsed+crossing, 'voltage_v': source})
            if 0 < crossing < duration:
                internal.append((crossing, source))
        if slope > 0 and begin_surplus <= available < surplus:
            crossing = (available-begin_surplus)/slope
            events.append({'segment_index': index, 'event': 'maximum_voltage_crossed_up',
                           'time_s': elapsed+crossing, 'voltage_v': upper})
            if 0 < crossing < duration:
                internal.append((crossing, upper))
        if slope < 0 and surplus < available <= begin_surplus:
            crossing = (begin_surplus-available)/(-slope)
            events.append({'segment_index': index, 'event': 'maximum_voltage_crossed_down',
                           'time_s': elapsed+crossing, 'voltage_v': upper})
            if 0 < crossing < duration:
                internal.append((crossing, upper))
        for local_time, value in sorted(internal):
            time_trace.append(elapsed+local_time)
            voltage_trace.append(value)
        if surplus > peak_surplus:
            peak_surplus, peak_time = surplus, elapsed+duration
        rows.append({'segment_index': index, 'begin_time_s': elapsed, 'end_time_s': elapsed+duration,
                     'duration_s': duration, 'motor_electrical_power_w': motor_power,
                     'dc_power_w': dc_power, 'inverter_loss_w': inverter_loss,
                     'net_capacitor_power_before_floor_w': slope,
                     'regenerated_dc_power_w': regeneration,
                     'begin_voltage_v': begin_voltage, 'end_voltage_v': voltage(surplus),
                     'begin_excess_capacitor_energy_j': begin_surplus,
                     'end_excess_capacitor_energy_j': surplus,
                     'source_delivered_energy_j': supply_energy,
                     'externally_absorbed_energy_j': max(absorbed_energy, 0.0),
                     'maximum_external_absorbed_power_w': (
                         absorption if begin_surplus > 0 else min(absorption, regeneration)) if dc_power < 0 else 0.0,
                     'capacitor_reused_for_motoring_energy_j': reused_energy,
                     'energy_balance_residual_j': residual})
        elapsed += duration
        time_trace.append(elapsed)
        voltage_trace.append(voltage(surplus))

    total = lambda key: math.fsum(row[key] for row in rows)
    dc_energy = math.fsum(row['dc_power_w']*row['duration_s'] for row in rows)
    source_energy = total('source_delivered_energy_j')
    absorbed_energy = total('externally_absorbed_energy_j')
    gross_regen = math.fsum(row['regenerated_dc_power_w']*row['duration_s'] for row in rows)
    max_voltage = voltage(peak_surplus)
    energy_capacity_per_f = (upper-source)*(upper+source)/2
    return {
        'model': 'piecewise_constant_power_ideal_source_and_limited_regenerative_sink',
        'implementation': '新写的母线分段能量初筛，不是驱动器控制或故障响应仿真',
        'parameters': {'capacitance_f': capacitance, 'source_voltage_v': source,
                       'minimum_voltage_v': lower, 'maximum_voltage_v': upper,
                       'initial_voltage_v': initial, 'inverter_motoring_efficiency': eta_m,
                       'inverter_regenerating_efficiency': eta_r, 'absorption_power_limit_w': absorption},
        'segments': rows, 'events': events, 'time_s': time_trace, 'voltage_v': voltage_trace,
        'total_duration_s': elapsed, 'maximum_voltage_v': max_voltage,
        'minimum_voltage_v': min(voltage_trace), 'maximum_voltage_time_s': peak_time,
        'maximum_regenerated_dc_power_w': max(row['regenerated_dc_power_w'] for row in rows),
        'maximum_motoring_dc_power_w': max(max(row['dc_power_w'], 0.0) for row in rows),
        'regenerated_dc_energy_j': gross_regen,
        'signed_dc_energy_j': dc_energy,
        'inverter_loss_energy_j': math.fsum(row['inverter_loss_w']*row['duration_s'] for row in rows),
        'source_delivered_energy_j': source_energy, 'externally_absorbed_energy_j': absorbed_energy,
        'average_external_absorbed_power_w': absorbed_energy/elapsed,
        'maximum_external_absorbed_power_w': max(row['maximum_external_absorbed_power_w'] for row in rows),
        'capacitor_reused_for_motoring_energy_j': total('capacitor_reused_for_motoring_energy_j'),
        'maximum_excess_capacitor_energy_j': peak_surplus,
        'capacitor_energy_capacity_to_maximum_voltage_j': available,
        'additional_peak_energy_buffer_required_j': max(peak_surplus-available, 0.0),
        'minimum_capacitance_f_for_nominal_start': reference_peak/energy_capacity_per_f,
        'final_excess_capacitor_energy_j': surplus,
        'maximum_segment_energy_residual_j': max(abs(row['energy_balance_residual_j']) for row in rows),
        'total_energy_balance_residual_j': surplus-initial_surplus+dc_energy+absorbed_energy-source_energy,
        'within_maximum_voltage': max_voltage <= upper*(1+1e-12),
        'within_minimum_voltage_by_ideal_source_assumption': min(voltage_trace) >= lower,
        'scope': '分段恒功率与理想无限供能电源；回馈吸收能力显式给定；不含PWM、阻抗、故障切断、制动电阻、电池充电和真实驱动器电压/电流包络',
        'minimum_voltage_scope': '最低电压由理想源电压下限保证，不构成有限电源能力或实物欠压校核',
        'capacitance_scope': '最低电容对应源电压起始、同一分段功率/吸收模型的一次输入历程，不含器件额定值裕量或重复任务自动平衡',
        'physical_validation': False,
    }
