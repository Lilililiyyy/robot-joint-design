"""新写的电机—传动双节点线性热网络，SI单位为s、W、J/K、W/K和K。"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np


def _scalar(value, name, *, positive=False, nonnegative=False):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(float(value)):
        raise ValueError(f"{name}必须为有限实数")
    value = float(value)
    if positive and value <= 0:
        raise ValueError(f"{name}必须大于0")
    if nonnegative and value < 0:
        raise ValueError(f"{name}不能为负")
    return value


def _pair(values, name, *, positive=False, nonnegative=False):
    raw = np.asarray(values)
    if raw.shape != (2,) or raw.dtype.kind not in "iuf":
        raise ValueError(f"{name}必须为长度2的实数数组")
    return np.array([_scalar(v, name, positive=positive, nonnegative=nonnegative) for v in raw])


def _network_parameters(capacities_j_per_k, ambient_conductances_w_per_k,
                        coupling_w_per_k, ambient_temperature_k):
    """验证热网络并构造热容对称化的两个衰减模态。"""
    capacities = _pair(capacities_j_per_k, "热容J/K", positive=True)
    ambient_conductance = _pair(ambient_conductances_w_per_k, "对环境热导W/K", nonnegative=True)
    coupling = _scalar(coupling_w_per_k, "节点间热导W/K", nonnegative=True)
    ambient = _scalar(ambient_temperature_k, "环境温度K", positive=True)
    conductance = np.diag(ambient_conductance + coupling) + np.array([[0.0, -coupling], [-coupling, 0.0]])
    roots = np.sqrt(capacities)
    normalized = conductance / np.outer(roots, roots)
    eigenvalues, eigenvectors = np.linalg.eigh(normalized)
    if eigenvalues[0] <= 0:
        raise ValueError("热网络须经正热导连接环境，每个节点均须存在散热路径")
    return capacities, ambient_conductance, ambient, conductance, roots, eigenvalues, eigenvectors


def _loss_segments(segments):
    """解析分段恒损耗；两个节点的损耗均为不可逆发热功率。"""
    if not isinstance(segments, (list, tuple)) or not segments:
        raise ValueError("热工况必须为非空片段列表")
    result = []
    for segment in segments:
        if not isinstance(segment, dict) or not {"duration_s", "motor_loss_w", "gear_loss_w"}.issubset(segment):
            raise ValueError("热片段须含duration_s、motor_loss_w和gear_loss_w")
        duration = _scalar(segment["duration_s"], "片段持续时间s", positive=True)
        loss = np.array([_scalar(segment["motor_loss_w"], "电机损耗W", nonnegative=True),
                         _scalar(segment["gear_loss_w"], "传动损耗W", nonnegative=True)])
        result.append((duration, loss))
    return result


def thermal_response(segments, *, capacities_j_per_k, ambient_conductances_w_per_k,
                     coupling_w_per_k, ambient_temperature_k,
                     initial_temperature_k=None) -> dict:
    """对分段恒定损耗求双节点精确瞬态和段内峰温。

    C θdot=P-G θ，θ=T-Tamb；G=[[Ga,m+Gc,-Gc],[-Gc,Ga,g+Gc]]。
    通过C^(-1/2)G C^(-1/2)对称特征分解计算矩阵指数，不采用时间步近似。
    两实指数的导数最多有一个内部零点，逐节点检查该点及端点得到峰温。
    热容、热导和损耗均为外部输入；不计算电磁场、润滑或自然对流系数。
    """
    (capacities, ambient_conductance, ambient, conductance,
     roots, eigenvalues, eigenvectors) = _network_parameters(
         capacities_j_per_k, ambient_conductances_w_per_k, coupling_w_per_k, ambient_temperature_k)
    initial = (np.full(2, ambient) if initial_temperature_k is None
               else _pair(initial_temperature_k, "初始温度K", positive=True))
    parsed_segments = _loss_segments(segments)
    temperatures = [initial.tolist()]
    times = [0.0]
    theta = initial - ambient
    maximum = initial.copy()
    maximum_time = np.zeros(2)
    steady_temperatures = []
    segment_results = []
    total_input = total_ambient = total_stored = 0.0
    elapsed = 0.0
    for duration, loss in parsed_segments:
        equilibrium = np.linalg.solve(conductance, loss)
        modes = eigenvectors.T @ (roots * (theta - equilibrium))
        coefficients = eigenvectors / roots[:, None] * modes[None, :]

        def value_at(time):
            return equilibrium + coefficients @ np.exp(-eigenvalues * time)

        candidates = {0.0, duration}
        for node in range(2):
            derivative_modes = -eigenvalues * coefficients[node]
            if derivative_modes[0] * derivative_modes[1] < 0:
                difference = eigenvalues[1] - eigenvalues[0]
                if difference > np.finfo(float).eps * eigenvalues[1] * 8:
                    stationary = (math.log(abs(derivative_modes[1])) - math.log(abs(derivative_modes[0]))) / difference
                    if 0 < stationary < duration:
                        candidates.add(float(stationary))
        for time in candidates:
            absolute_temperature = value_at(time) + ambient
            greater = absolute_temperature > maximum
            maximum[greater] = absolute_temperature[greater]
            maximum_time[greater] = elapsed + time
        sample_times = sorted(candidates | set(np.linspace(0.0, duration, 17).tolist()))
        for time in sample_times[1:]:
            times.append(elapsed + time)
            temperatures.append((value_at(time) + ambient).tolist())
        end_theta = value_at(duration)
        integral_theta = equilibrium * duration + coefficients @ (-np.expm1(-eigenvalues * duration) / eigenvalues)
        input_energy = float(loss.sum() * duration)
        ambient_energy = float(np.dot(ambient_conductance, integral_theta))
        stored_energy = float(np.dot(capacities, end_theta - theta))
        residual = stored_energy - input_energy + ambient_energy
        segment_results.append({"duration_s": duration, "motor_loss_w": float(loss[0]),
                                "gear_loss_w": float(loss[1]),
                                "energy_input_j": input_energy, "heat_to_ambient_j": ambient_energy,
                                "stored_energy_change_j": stored_energy, "energy_residual_j": residual})
        total_input += input_energy
        total_ambient += ambient_energy
        total_stored += stored_energy
        theta = end_theta
        elapsed += duration
        steady_temperatures.append((equilibrium + ambient).tolist())
    return {
        "time_s": times,
        "temperature_k": temperatures,
        "final_temperature_k": (theta + ambient).tolist(),
        "maximum_temperature_k": maximum.tolist(),
        "maximum_time_s": maximum_time.tolist(),
        "steady_temperature_k_per_segment": steady_temperatures,
        "segments": segment_results,
        "diagnostics": {"energy_input_j": total_input, "heat_to_ambient_j": total_ambient,
                        "stored_energy_change_j": total_stored,
                        "energy_residual_j": total_stored - total_input + total_ambient,
                        "exact_segment_propagation": True,
                        "interior_stationary_points_checked": True},
        "model": "分段恒定损耗、恒定热容和热导的双节点线性热网络",
        "physical_validation": False,
    }


def periodic_thermal_response(segments, *, capacities_j_per_k, ambient_conductances_w_per_k,
                              coupling_w_per_k, ambient_temperature_k) -> dict:
    """求无限重复给定工作循环后的周期热稳态，再分析完整一周期。

    先对分段恒损耗构造θend=Acycle θ0+b，并解(I-Acycle)θ0=b。
    在热容对称模态中Acycle对角化，采用-expm1(-λT)计算1-exp(-λT)，
    避免短工作循环相对长热时间常数时的消减误差。随后复用单周期解析
    峰温和能量核验。此函数不接收一次冷启动初温，不把一次运行当稳态。
    """
    (_, _, ambient, _, roots, eigenvalues, eigenvectors) = _network_parameters(
        capacities_j_per_k, ambient_conductances_w_per_k, coupling_w_per_k, ambient_temperature_k)
    parsed_segments = _loss_segments(segments)
    # z=Vᵀ C^(1/2)θ，每个模态满足zdot=-λz+Vᵀ C^(-1/2)P。
    forcing_from_zero = np.zeros(2)
    duration_total = 0.0
    for duration, loss in parsed_segments:
        decay = np.exp(-eigenvalues * duration)
        forcing = eigenvectors.T @ (loss / roots)
        forcing_from_zero = decay * forcing_from_zero + forcing * (-np.expm1(-eigenvalues * duration)) / eigenvalues
        duration_total += duration
    modal_initial = forcing_from_zero / (-np.expm1(-eigenvalues * duration_total))
    initial = ambient + eigenvectors @ modal_initial / roots
    result = thermal_response(
        segments, capacities_j_per_k=capacities_j_per_k,
        ambient_conductances_w_per_k=ambient_conductances_w_per_k,
        coupling_w_per_k=coupling_w_per_k, ambient_temperature_k=ambient_temperature_k,
        initial_temperature_k=initial)
    residual = np.asarray(result["final_temperature_k"]) - initial
    result.update({
        "periodic_steady": True,
        "periodic_initial_temperature_k": initial.tolist(),
        "periodic_residual_k": float(np.max(np.abs(residual))),
        "periodic_residual_per_node_k": residual.tolist(),
        "period_duration_s": duration_total,
    })
    result["diagnostics"]["periodic_boundary_residual_k"] = result["periodic_residual_k"]
    result["model"] = "分段恒损耗双节点线性热网络的周期稳态；解析周期初值与段内峰温"
    return result
