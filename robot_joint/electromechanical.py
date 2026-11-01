"""新写的关节机电匹配模型；扭矩N·m、角速度rad/s、时间s、功率W。

本模块采用给定正向/回驱效率、分段代表工况和电机恒转矩—恒功率包络。
它不预测齿面摩擦，不模拟电流环，不替代电机热认证或制动器动态认证。
电机参数Kt和R必须采用同一电流基准，R为使铜耗满足I²R的等效电阻。
"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
import math
from numbers import Real


def _number(value, name, *, minimum=None, strict=False):
    """校验米制标量，不将布尔值当作数值参数。"""
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name}必须为有限实数")
    result = float(value)
    if minimum is not None and (result <= minimum if strict else result < minimum):
        relation = "大于" if strict else "不小于"
        raise ValueError(f"{name}必须{relation}{minimum}")
    return result


def _efficiency(value, name):
    result = _number(value, name, minimum=0, strict=True)
    if result > 1:
        raise ValueError(f"{name}必须位于(0,1]")
    return result


def motor_duty(segments, ratio, *, motor, forward_efficiency,
               backdrive_efficiency, brake_torque_n_m, brake_safety_factor=1.5):
    """由输出端代表工况校核太阳轮输入电机和输入侧静态制动器。

    ratio=ωmotor/ωoutput>1。输出扭矩为关节对负载施加的有符号扭矩；
    T·ω>0为驱动，T·ω<0为回驱。驱动负载力矩反射为T/(iηf)，
    回驱为Tηb/i；零速使用T/i，未预测静摩擦及启动峰值。
    电机力矩再加Jmotor·i·αoutput，不包含未给出的转子外输入侧惯量。

    每段速度和加速度为调用方给定的代表值，不由不连续段推导加速度，
    不强制它们组成完整积分轨迹。时长用于RMS、能量和热输入加权。
    连续工作初筛采用按转速包络归一化的转矩RMS；其不是电机热模型。
    """
    ratio = _number(ratio, "ratio", minimum=1, strict=True)
    eta_forward = _efficiency(forward_efficiency, "forward_efficiency")
    eta_back = _efficiency(backdrive_efficiency, "backdrive_efficiency")
    brake_torque = _number(brake_torque_n_m, "brake_torque_n_m", minimum=0)
    brake_factor = _number(brake_safety_factor, "brake_safety_factor", minimum=1)
    if not isinstance(motor, Mapping):
        raise ValueError("motor必须为包含电机参数的对象")
    required = {
        "torque_constant_n_m_per_a", "resistance_ohm", "continuous_torque_n_m",
        "peak_torque_n_m", "base_speed_rad_s", "maximum_speed_rad_s",
        "rotor_inertia_kg_m2",
    }
    optional = {"no_load_loss_w", "speed_loss_w_per_rad_s_squared"}
    missing, unknown = required-set(motor), set(motor)-required-optional
    if missing:
        raise ValueError("motor缺少参数："+", ".join(sorted(missing)))
    if unknown:
        raise ValueError("motor含未知参数："+", ".join(sorted(unknown)))
    parameters = {
        name: _number(motor[name], name, minimum=0,
                      strict=name not in {"resistance_ohm", "rotor_inertia_kg_m2"})
        for name in required
    }
    for name in optional:
        parameters[name] = _number(motor.get(name, 0), name, minimum=0)
    if parameters["peak_torque_n_m"] < parameters["continuous_torque_n_m"]:
        raise ValueError("峰值转矩不能小于连续转矩")
    if parameters["maximum_speed_rad_s"] < parameters["base_speed_rad_s"]:
        raise ValueError("最大转速不能小于基速")
    if not isinstance(segments, Sequence) or isinstance(segments, (str, bytes)) or not segments:
        raise ValueError("segments必须为非空工况序列")

    results = []
    allowed = {"duration_s", "output_torque_n_m", "output_speed_rad_s",
               "output_acceleration_rad_s2"}
    for index, segment in enumerate(segments):
        if not isinstance(segment, Mapping):
            raise ValueError(f"工况{index}必须为对象")
        missing = {"duration_s", "output_torque_n_m", "output_speed_rad_s"}-set(segment)
        if missing or set(segment)-allowed:
            raise ValueError(f"工况{index}缺少必要字段或含未知字段")
        duration = _number(segment["duration_s"], "duration_s", minimum=0, strict=True)
        torque = _number(segment["output_torque_n_m"], "output_torque_n_m")
        speed = _number(segment["output_speed_rad_s"], "output_speed_rad_s")
        acceleration = _number(segment.get("output_acceleration_rad_s2", 0),
                               "output_acceleration_rad_s2")
        output_power = torque*speed
        motor_speed = ratio*speed
        if speed == 0:
            mode = "holding"
            reflected = torque/ratio
            gear_loss = 0.0
        elif output_power >= 0:
            mode = "motoring"
            reflected = torque/(ratio*eta_forward)
            gear_loss = output_power*(1/eta_forward-1)
        else:
            mode = "backdriving"
            reflected = torque*eta_back/ratio
            gear_loss = -output_power*(1-eta_back)
        inertia_torque = parameters["rotor_inertia_kg_m2"]*ratio*acceleration
        motor_torque = reflected+inertia_torque
        current = motor_torque/parameters["torque_constant_n_m_per_a"]
        copper_loss = current*current*parameters["resistance_ohm"]
        other_loss = (parameters["no_load_loss_w"]
                      + parameters["speed_loss_w_per_rad_s_squared"]*motor_speed*motor_speed)
        motor_loss = copper_loss+other_loss
        mechanical_power = motor_torque*motor_speed
        inertia_power = inertia_torque*motor_speed
        gearbox_input_power = reflected*motor_speed
        electrical_power = mechanical_power+motor_loss
        envelope_scale = (1.0 if abs(motor_speed) <= parameters["base_speed_rad_s"]
                          else parameters["base_speed_rad_s"]/abs(motor_speed))
        continuous_limit = parameters["continuous_torque_n_m"]*envelope_scale
        peak_limit = parameters["peak_torque_n_m"]*envelope_scale
        speed_ok = abs(motor_speed) <= parameters["maximum_speed_rad_s"]
        peak_ok = abs(motor_torque) <= peak_limit
        continuous_ok = abs(motor_torque) <= continuous_limit
        results.append({
            "duration_s": duration, "mode": mode,
            "output_torque_n_m": torque, "output_speed_rad_s": speed,
            "output_acceleration_rad_s2": acceleration,
            "motor_speed_rad_s": motor_speed, "motor_acceleration_rad_s2": ratio*acceleration,
            "motor_torque_n_m": motor_torque, "reflected_load_torque_n_m": reflected,
            "rotor_inertia_torque_n_m": inertia_torque, "motor_current_a": current,
            "output_power_w": output_power, "gearbox_input_power_w": gearbox_input_power,
            "motor_mechanical_power_w": mechanical_power, "rotor_inertia_power_w": inertia_power,
            "electrical_power_w": electrical_power, "gear_loss_w": gear_loss,
            "copper_loss_w": copper_loss, "other_motor_loss_w": other_loss,
            "motor_loss_w": motor_loss, "continuous_torque_limit_n_m": continuous_limit,
            "peak_torque_limit_n_m": peak_limit,
            "continuous_torque_utilization": abs(motor_torque)/continuous_limit,
            "within_speed_limit": speed_ok, "within_peak_torque_limit": peak_ok,
            "within_continuous_torque_limit": continuous_ok,
            "gearbox_power_residual_w": gearbox_input_power-output_power-gear_loss,
            "power_residual_w": electrical_power-output_power-inertia_power-gear_loss-motor_loss,
        })
    total_duration = sum(item["duration_s"] for item in results)
    rms_current = math.sqrt(sum(item["motor_current_a"]**2*item["duration_s"]
                                for item in results)/total_duration)
    rms_torque = math.sqrt(sum(item["motor_torque_n_m"]**2*item["duration_s"]
                               for item in results)/total_duration)
    normalized_rms = math.sqrt(sum(item["continuous_torque_utilization"]**2*item["duration_s"]
                                    for item in results)/total_duration)
    brake_required = max(abs(item["output_torque_n_m"]) for item in results)/ratio*brake_factor
    energy_names = {
        "electrical_energy_j": "electrical_power_w", "output_energy_j": "output_power_w",
        "rotor_inertia_energy_j": "rotor_inertia_power_w", "gear_loss_energy_j": "gear_loss_w",
        "motor_loss_energy_j": "motor_loss_w",
    }
    energy = {name: sum(item[source]*item["duration_s"] for item in results)
              for name, source in energy_names.items()}
    return {
        "model": "given_efficiency_and_constant_power_motor_envelope",
        "implementation": "新写的分段代表工况机电匹配模型", "ratio": ratio,
        "segments": results, "total_duration_s": total_duration,
        "rms_current_a": rms_current, "rms_motor_torque_n_m": rms_torque,
        "normalized_continuous_torque_rms": normalized_rms,
        "within_speed_limit": all(item["within_speed_limit"] for item in results),
        "within_peak_torque_limit": all(item["within_peak_torque_limit"] for item in results),
        "continuous_duty_screen_pass": normalized_rms <= 1,
        "brake_required_torque_n_m": brake_required,
        "brake_available_torque_n_m": brake_torque, "brake_safety_factor": brake_factor,
        "brake_static_screen_pass": brake_torque >= brake_required,
        "energy": energy,
        "maximum_power_residual_w": max(abs(item["power_residual_w"]) for item in results),
        "scope": "给定效率与分段代表工况；连续转矩RMS仅为初筛，制动器仅校核输入侧静态保持，零速静摩擦未预测",
        "physical_validation": False,
    }
