"""结果对标工具：计算MASTA单副对标及公开硬件参数的一致性。

参照响应只用于比较，不进入求解；数值接近不等于整机关节物理验证。
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

PROJECT = next(p for p in Path(__file__).resolve().parents
               if (p / "pyproject.toml").is_file() and (p / "robot_joint").is_dir())
if str(PROJECT) not in sys.path:
    sys.path.insert(0, str(PROJECT))

from gear_contact import Material, analyze_config
from gear_contact.io import write_json
from gear_contact.strength import (
    MeshRatingFactors, GearRatingFactors, StressLimits,
    evaluate_iso6336_explicit_factors,
)
from robot_joint.fatigue import rainflow_cycles


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_csv(path, rows):
    """保留SI列名、原数值和空相对差，不做图形归一化替代原数组。"""
    with Path(path).open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def calculate_masta_case(case):
    """只由独立几何、载荷、材料、修形与显式评级输入计算。"""
    config = deepcopy(case["analysis"])
    loading = case["loading"]
    nominal = loading["nominal_pinion_torque_n_m"]
    config["torque_n_m"] = nominal * loading["ltca_torque_multiplier"]
    settings = dict(config.get("iso_settings", {}))
    settings["nominal_pinion_torque_n_m"] = nominal
    config["iso_settings"] = settings
    result = analyze_config(config)
    parameters = deepcopy(case["strength"])
    parameters["nominal_pinion_torque_n_m"] = nominal
    parameters["mesh_factors"] = MeshRatingFactors(**parameters["mesh_factors"])
    for role in ("pinion", "wheel"):
        parameters[role + "_factors"] = GearRatingFactors(**parameters[role + "_factors"])
        parameters[role + "_limits"] = StressLimits(**parameters[role + "_limits"])
        parameters["material_" + role] = Material(**config.get("material_" + role, {}))
    strength = evaluate_iso6336_explicit_factors(result.pair, **parameters)
    return result, strength


def _field(value, field):
    for key in field.split("."):
        value = value[key] if isinstance(value, dict) else getattr(value, key)
    return float(value)


def comparison(group, field, actual, reference, *, atol=None, rtol=None):
    """共同公式量可判数值通过，模型响应仅报告差值而不虚设精度指标。"""
    actual, reference = float(actual), float(reference)
    if not math.isfinite(actual) or not math.isfinite(reference):
        raise ValueError("对标数值须有限")
    error = abs(actual - reference)
    return {"group": group, "field": field, "project": actual, "reference": reference,
            "absolute_error": error,
            "relative_error": error / abs(reference) if reference != 0 else None,
            "atol": atol, "rtol": rtol,
            "matched": (error <= atol + rtol * abs(reference)
                        if atol is not None and rtol is not None else None)}


def run_masta(reference_dir, output):
    """复算32相位，绑定文件身份，逐相位输出法向TE和刚度。"""
    provenance = read(reference_dir / "provenance.json")
    for filename, expected in provenance["files"].items():
        path = (reference_dir / filename).resolve()
        if reference_dir not in path.parents or digest(path) != expected:
            raise ValueError("MASTA参照文件身份不一致，拒绝继承对标状态")
    for filename, expected in provenance["kernel_sha256"].items():
        if digest(PROJECT / "gear_contact" / filename) != expected:
            raise ValueError("齿轮内核已变化，须重新审查参照模型映射后更新身份记录")
    case = read(reference_dir / "case.json")
    reference = read(reference_dir / "reference.json")
    result, strength = calculate_masta_case(case)
    if (not result.summary["all_converged"]
            or result.summary["max_force_residual_n"] > 1e-7
            or result.summary["max_gap_residual_m"] > 3e-12):
        raise RuntimeError("MASTA对标案例自身求解残差未达要求")
    rows = []
    for field, original in reference["geometry_values"].items():
        actual = result.summary["normal_force_n"] if field == "normal_force_n" else _field(result.pair, field)
        rows.append(comparison("geometry", field, actual, original, atol=1e-10, rtol=1e-8))
    for field, original in reference["strength_values"].items():
        rows.append(comparison("strength", field, _field(strength, field), original,
                               atol=1e-5 if field.endswith("_pa") else 1e-8, rtol=2e-8))
    for field, original in reference["ltca_values"].items():
        rows.append(comparison("ltca", field, result.summary[field], original))
    with np.load(reference_dir / "contact.npz", allow_pickle=False) as source:
        arrays = {key: source[key].copy() for key in source.files}
    phase_error = float(np.max(np.abs((result.phase - arrays["rotation_phase"] + .5) % 1 - .5)))
    if phase_error > 1e-10:
        raise ValueError("同几何相位配准未通过，不得直接比较逐相位响应")
    cycles = [{"phase_cycles": float(result.phase[i]),
               "project_te_m": float(result.normal_approach_m[i]),
               "reference_te_m": float(arrays["rotation_te_normal_m"][i]),
               "project_stiffness_n_per_m": float(result.mesh_stiffness_n_per_m[i]),
               "reference_stiffness_n_per_m": float(arrays["rotation_stiffness_n_per_m"][i])}
              for i in range(len(result.phase))]
    nodes, geometry_error, masks = [], 0., 0
    for line, rotation in enumerate(arrays["line_rotation_index"]):
        current = result.rotations[int(rotation)]
        pair_index = arrays["line_project_tooth_pair_index"][line]
        mask = np.asarray(current.grid.tooth_pair_index) == pair_index
        start, end = arrays["centre_offsets"][line:line + 2]
        source_slice = slice(int(start), int(end))
        if int(np.sum(mask)) != end - start:
            raise ValueError("当前条带布局不等于参照，拒绝强行插值对标")
        face_error = np.max(np.abs(current.grid.face_coordinate_m[mask] - arrays["centre_face_coordinate_m"][source_slice]))
        roll_error = np.max(np.abs(current.grid.roll_pinion_m[mask] - arrays["centre_roll_pinion_m"][source_slice]))
        geometry_error = max(geometry_error, float(face_error), float(roll_error))
        if geometry_error > 1e-11:
            raise ValueError("同齿对节点坐标不一致，拒绝把相邻节点响应视为同点")
        project_force = current.solution.forces_n[mask]
        reference_force = arrays["centre_force_n"][source_slice]
        masks += int(np.count_nonzero((project_force > 1e-10) != (reference_force > 1e-10)))
        nodes.extend({"rotation_index": int(rotation), "line_index": line,
                      "tooth_pair_index": int(pair_index),
                      "face_coordinate_m": float(face), "roll_pinion_m": float(roll),
                      "project_force_n": float(own), "reference_force_n": float(original)}
                     for face, roll, own, original in zip(current.grid.face_coordinate_m[mask],
                         current.grid.roll_pinion_m[mask], project_force, reference_force))
    te_error = np.abs(result.normal_approach_m - arrays["rotation_te_normal_m"])
    stiffness_error = np.abs(result.mesh_stiffness_n_per_m - arrays["rotation_stiffness_n_per_m"])
    source_force = np.array([row["reference_force_n"] for row in nodes])
    own_force = np.array([row["project_force_n"] for row in nodes])
    summary = {"reference_name": "MASTA 15.0.179.93", "case_id": case["case_id"],
               "own_values_recomputed": True, "reference_response_used_as_input": False,
               "reference_identity_verified": True, "vendor_runtime_required": False,
               "geometry_fields": sum(row["group"] == "geometry" for row in rows),
               "strength_fields": sum(row["group"] == "strength" for row in rows),
               "common_fields_matched": sum(row["matched"] is True for row in rows),
               "common_comparison_passed": all(row["matched"] is True for row in rows if row["group"] != "ltca"),
               "ltca_fields": sum(row["group"] == "ltca" for row in rows),
               "phase_count": len(cycles), "strip_centres": len(nodes),
               "phase_residual_cycles": phase_error, "node_coordinate_residual_m": geometry_error,
               "maximum_te_absolute_error_m": float(np.max(te_error)),
               "maximum_te_error_over_reference_mean": float(np.max(te_error) / abs(np.mean(arrays["rotation_te_normal_m"]))),
               "maximum_stiffness_relative_error": float(np.max(stiffness_error / np.abs(arrays["rotation_stiffness_n_per_m"]))),
               "maximum_strip_force_absolute_error_n": float(np.max(np.abs(own_force - source_force))),
               "maximum_strip_force_error_over_reference_peak": float(np.max(np.abs(own_force - source_force)) / np.max(np.abs(source_force))),
               "contact_mask_mismatches_at_1e_minus_10_n": masks,
               "summary": result.summary,
               "independent_rating_factor_selection": False,
               "ltca_model_equivalence_claimed": False,
               "ltca_engineering_accuracy_verified": False,
               "whole_joint_vendor_benchmark": False, "physical_validation": False}
    write_csv(output / "comparison.csv", rows)
    write_csv(output / "cycle.csv", cycles)
    write_csv(output / "strip_forces.csv", nodes)
    write_json(output / "comparison.json", rows)
    write_json(output / "masta_summary.json", summary)
    return summary


def run_rainflow(reference_path, output):
    """对固定循环计数样例逐行核验range、mean、count。"""
    reference = read(reference_path)
    rows = []
    for case in reference["cases"]:
        actual = rainflow_cycles(case["input"])
        if len(actual) != len(case["expected"]):
            raise ValueError("循环计数行数不等于固定参考")
        for index, (row, expected) in enumerate(zip(actual, case["expected"])):
            for field, original in zip(reference["columns"], expected):
                rows.append({"case_id": case["case_id"], "cycle_index": index,
                             **comparison("rainflow", field, row[field], original, atol=0., rtol=0.)})
    summary = {"cases": len(reference["cases"]), "cycle_rows": len(rows) // 3,
               "scalar_fields": len(rows), "matched_fields": sum(row["matched"] for row in rows),
               "all_matched": all(row["matched"] for row in rows),
               "numerical_regression": True,
               "reference_sha256": digest(reference_path), "physical_validation": False}
    write_csv(output / "rainflow_comparison.csv", rows)
    return summary


def run_hardware(output):
    """仅核对公开实物参数与简化质量分布，不声称实际产品性能误差。"""
    requirements = read(PROJECT / "examples/dfki_pendulum_requirements.json")
    case = read(PROJECT / "examples/dfki_pendulum_case.json")
    p = requirements["pendulum"]
    total = p["point_mass_kg"] + p["rod_mass_kg"]
    com = (p["point_mass_kg"] + .5 * p["rod_mass_kg"]) * p["length_m"] / total
    stage = case["stage"]
    rows = [comparison("hardware_parameter", "total_mass_kg", total, .676, atol=1e-12, rtol=0.),
            comparison("hardware_parameter", "uniform_rod_com_m", com, p["published_com_m"]),
            comparison("hardware_parameter", "gear_ratio", 1 + stage["ring_teeth"] / stage["sun_teeth"], 6., atol=0., rtol=0.),
            comparison("hardware_parameter", "motor_rated_torque_from_kt_n_m", case["motor"]["torque_constant_n_m_per_a"] * 12., 1.092, atol=1e-12, rtol=0.),
            comparison("hardware_parameter", "motor_peak_torque_from_kt_n_m", case["motor"]["torque_constant_n_m_per_a"] * 24., 2.184, atol=1e-12, rtol=0.),
            comparison("source_conflict", "sun_planet_center_distance_m", stage["module_m"] * (stage["sun_teeth"] + stage["planet_teeth"]) / 2,
                       requirements["source_conflicts"]["mjbots_published_center_distance_m"])]
    summary = {"parameter_rows": len(rows), "exact_consistency_checks": 4,
               "exact_consistency_matched": sum(row["matched"] is True for row in rows),
               "uniform_rod_com_relative_difference": rows[1]["relative_error"],
               "source_conflicts": 1, "scope_zh": "公开硬件参数一致性；质心是简化模型与近似公开数值之差，中心距是未解决来源冲突；没有实测性能比较。",
               "hardware_performance_validation": False}
    write_csv(output / "hardware_parameters.csv", rows)
    return summary


def run(output):
    """输出新求解数据，禁止覆盖离线参照目录。"""
    output = Path(output).resolve()
    reference = (PROJECT / "benchmarks/reference").resolve()
    if output == reference or reference in output.parents or output in reference.parents:
        raise ValueError("结果目录不得覆盖参照目录及其父子路径")
    output.mkdir(parents=True, exist_ok=True)
    masta = run_masta(reference / "masta_18_77", output)
    hardware = run_hardware(output)
    summary = {"schema_version": 1, "masta": masta, "hardware": hardware,
               "physical_validation": False, "whole_joint_vendor_benchmark": False,
               "passed": masta["common_comparison_passed"] and hardware["exact_consistency_matched"] == 4,
               "validation_tool_sha256": digest(Path(__file__))}
    write_json(output / "summary.json", summary)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=PROJECT / "output/benchmarks", help="复算结果目录")
    args = parser.parse_args()
    result = run(args.output)
    print(json.dumps({"passed": result["passed"], "masta_common_fields": result["masta"]["common_fields_matched"],
                      "physical_validation": False}, ensure_ascii=False, indent=2))
    if not result["passed"]:
        raise SystemExit("已声明判据的对标量未通过")


if __name__ == "__main__":
    main()
