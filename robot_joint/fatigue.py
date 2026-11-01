"""新写的一维雨流循环计数和显式S-N曲线Miner损伤计算。

输入可以是应力时程，或已经独立换算为应力的扭矩历程；不得把扭矩
单位直接代入Pa形式的S-N参数。本模块不自动推导齿轮受载循环次数。
"""
from __future__ import annotations

import math
from numbers import Real

import numpy as np


def _finite(value, name, *, positive=False):
    """检查S-N参数与循环表中的实数标量。"""
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, Real) or not math.isfinite(float(value)):
        raise ValueError(f"{name}必须为有限实数")
    value = float(value)
    if positive and value <= 0:
        raise ValueError(f"{name}必须大于0")
    return value


def rainflow_cycles(values) -> list[dict]:
    """用转折点栈输出range、mean和count，保留开序列的剩余半循环。

    相邻平台先压缩，再提取首尾点及峰谷。当前新范围不小于前范围时闭合
    前循环；触及序列起点计半循环，其余计整循环，结束后残余相邻点计半。
    空序列、单点和恒值返回空表。输入单位原样保留；没有流式残余状态接口。
    """
    raw = np.asarray(values)
    if raw.dtype.kind not in "iuf" or raw.ndim != 1:
        raise ValueError("雨流输入必须为一维实数序列")
    sequence = raw.astype(float)
    if not np.all(np.isfinite(sequence)):
        raise ValueError("雨流输入不能含NaN或无穷值")
    if sequence.size < 2:
        return []
    distinct = sequence[np.r_[True, np.diff(sequence) != 0]]
    if distinct.size < 2:
        return []
    differences = np.diff(distinct)
    reversals = distinct[np.r_[True, np.sign(differences[:-1]) != np.sign(differences[1:]), True]]
    stack = []
    cycles = []

    def emit(a, b, count):
        cycles.append({"range": abs(b - a), "mean": (a + b) / 2, "count": count})

    for value in reversals:
        stack.append(float(value))
        while len(stack) >= 3:
            previous_range = abs(stack[-2] - stack[-3])
            following_range = abs(stack[-1] - stack[-2])
            if previous_range > following_range:
                break
            if len(stack) == 3:
                emit(stack[0], stack[1], 0.5)
                del stack[0]
            else:
                emit(stack[-3], stack[-2], 1.0)
                del stack[-3:-1]
    for first, second in zip(stack[:-1], stack[1:]):
        emit(first, second, 0.5)
    return cycles


def sn_damage(cycles, *, reference_stress_pa, reference_cycles, slope,
              ultimate_strength_pa=None) -> dict:
    """按幅值幂律S-N和Miner线性累计计算一组应力循环的损伤。

    N(σa)=Nref(σref/σa)^m，D=Σcount/N。无耐久极限与多轴换算。
    ultimate_strength_pa给出时采用Goodman：σa,eq=σa/(1-max(σmean,0)/Su)，
    压缩均值不获有利修正；张拉均值达到Su的非零幅值循环显式拒绝。
    这是一种调用方选定的材料模型，不代表任意齿轮标准寿命评级。
    repeats_to_miner_one表示当前循环谱重复的次数；对截断时程，应先按
    实际顺序拼接后计数，不能把各段各自闭账的剩余半循环当作连续记录。
    """
    stress_reference = _finite(reference_stress_pa, "S-N参考幅值Pa", positive=True)
    cycle_reference = _finite(reference_cycles, "S-N参考循环数", positive=True)
    exponent = _finite(slope, "S-N斜率", positive=True)
    ultimate = (_finite(ultimate_strength_pa, "抗拉强度Pa", positive=True)
                if ultimate_strength_pa is not None else None)
    if not isinstance(cycles, (list, tuple)):
        raise ValueError("循环表必须为列表或元组")
    rows = []
    total = 0.0
    for cycle in cycles:
        if not isinstance(cycle, dict) or not {"range", "mean", "count"}.issubset(cycle):
            raise ValueError("循环表每项必须含range、mean和count")
        stress_range = _finite(cycle["range"], "应力范围Pa")
        mean = _finite(cycle["mean"], "均值应力Pa")
        count = _finite(cycle["count"], "循环计数")
        if stress_range < 0 or count < 0:
            raise ValueError("应力范围与循环计数不能为负")
        amplitude = stress_range / 2
        corrected = amplitude
        if ultimate is not None and amplitude > 0 and count > 0:
            if max(mean, 0.0) >= ultimate:
                raise ValueError("非零循环的张拉均值已达到抗拉强度，Goodman修正不适用")
            corrected /= 1 - max(mean, 0.0) / ultimate
        if corrected == 0 or count == 0:
            allowed, damage = None, 0.0
        else:
            log_allowed = math.log(cycle_reference) + exponent * (math.log(stress_reference) - math.log(corrected))
            # 极低幅值的理论循环数可超过浮点范围；损伤仍由对数式独立求值。
            allowed = math.exp(log_allowed) if log_allowed < math.log(np.finfo(float).max) else None
            log_damage = math.log(count) - log_allowed
            if log_damage > math.log(np.finfo(float).max):
                raise ValueError("S-N参数导致损伤超过可计算数值范围")
            damage = math.exp(log_damage)
        total += damage
        rows.append({"range_pa": stress_range, "mean_pa": mean, "count": count,
                     "amplitude_pa": amplitude, "corrected_amplitude_pa": corrected,
                     "predicted_cycles": allowed, "damage": damage})
    if not math.isfinite(total):
        raise ValueError("累计损伤超过可计算数值范围")
    return {
        "damage_per_duty": total,
        "repeats_to_miner_one": 1 / total if total > 0 else None,
        "cycles": rows,
        "reference_stress_pa": stress_reference,
        "reference_cycles": cycle_reference,
        "slope": exponent,
        "mean_stress_model": "Goodman张拉均值修正，压缩均值不获益" if ultimate is not None else "不作均值应力修正",
        "scope": "一维应力幅值幂律S-N与Miner累计；无耐久极限，不自动推导齿轮啮合循环",
        "repetition_scope": "按当前循环谱重复估算；非闭合时程应先拼接计数，不能逐段独立闭账后代替连续记录",
        "physical_validation": False,
    }
