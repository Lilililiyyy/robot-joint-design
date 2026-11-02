"""对标数据绑定、实际输入响应和固定循环表的回归核验。"""
from copy import deepcopy
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from examples.run_benchmarks import (
    PROJECT, calculate_masta_case, read, run, run_masta, run_rainflow,
)


class BenchmarkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.output = Path(cls.directory.name) / "results"
        cls.result = run(cls.output)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def test_masta_formula_fields_and_phase_geometry(self):
        result = self.result["masta"]
        self.assertTrue(result["common_comparison_passed"])
        self.assertEqual(result["common_fields_matched"], 26)
        self.assertEqual(result["phase_count"], 32)
        self.assertEqual(result["strip_centres"], 5040)
        self.assertLess(result["node_coordinate_residual_m"], 1e-11)
        self.assertLess(result["summary"]["max_force_residual_n"], 1e-7)
        self.assertFalse(result["whole_joint_vendor_benchmark"])
        self.assertFalse(result["ltca_engineering_accuracy_verified"])

    def test_reference_response_is_not_a_solver_argument(self):
        case = read(PROJECT / "benchmarks/reference/masta_18_77/case.json")
        modified = deepcopy(case)
        modified["loading"]["nominal_pinion_torque_n_m"] *= 1.5
        result, strength = calculate_masta_case(modified)
        baseline = self.result["masta"]["summary"]
        self.assertAlmostEqual(result.summary["normal_force_n"] / baseline["normal_force_n"], 1.5)
        self.assertGreater(result.summary["mean_normal_approach_m"], baseline["mean_normal_approach_m"])
        self.assertGreater(strength["pinion"]["contact_stress_pa"], 540463096.580786)

    def test_changed_reference_is_rejected_before_solving(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "reference"
            shutil.copytree(PROJECT / "benchmarks/reference/masta_18_77", target)
            case_path = target / "case.json"
            case = json.loads(case_path.read_text(encoding="utf-8"))
            case["loading"]["nominal_pinion_torque_n_m"] *= 2
            case_path.write_text(json.dumps(case), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "身份"):
                run_masta(target.resolve(), Path(directory))

    def test_output_cannot_replace_reference(self):
        reference = PROJECT / "benchmarks/reference"
        for output in (reference, reference / "new", reference.parent):
            with self.assertRaises(ValueError):
                run(output)

    def test_rainflow_fixed_cycle_tables(self):
        result = run_rainflow(PROJECT / "tests/fixtures/rainflow_cycles.json", self.output)
        self.assertEqual(result["cycle_rows"], 16)
        self.assertEqual(result["matched_fields"], 48)
        self.assertTrue(result["all_matched"])
        self.assertTrue(result["numerical_regression"])

    def test_hardware_consistency_is_separate_from_measurements(self):
        result = self.result["hardware"]
        self.assertEqual(result["exact_consistency_matched"], 4)
        self.assertEqual(result["source_conflicts"], 1)
        self.assertAlmostEqual(result["uniform_rod_com_relative_difference"], .004273504273504135)
        self.assertFalse(result["hardware_performance_validation"])
        self.assertFalse(self.result["physical_validation"])


if __name__ == "__main__":
    unittest.main()
