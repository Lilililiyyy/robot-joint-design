"""独立试次、夹具修正及与任务主流程采用相同静态转角的回归验证。"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from robot_joint import analyze_joint, calibrate_torsion, compare_torsion_model
from robot_joint.torsion_cli import run


ROOT = next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
DATA = ROOT/'examples/torsion_calibration'


def samples(levels, run_id, bias=0):
    rows = []
    for sign, stiffness in ((1, 18000), (-1, 20000)):
        for level in levels:
            torque = sign*level
            rows.append({'run_id': run_id, 'flank': 'positive' if sign > 0 else 'negative',
                         'torque_n_m': torque, 'angle_rad': .00013+sign*.00025+
                             torque/stiffness+torque*5e-6+sign*bias})
    return rows


class TorsionCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.fit = samples([1, 2, 3, 4], 'fit_run')
        self.verification = samples([1.5, 2.5, 3.5], 'verify_run', bias=4e-6)
        self.options = {'absolute_torque_window_n_m': [1, 5],
                        'fixture_compliance_rad_per_n_m': 5e-6, 'verification_limit_rad': 8e-6}

    def test_known_branch_stiffness_fixture_correction_and_held_out_fail(self):
        original = deepcopy((self.fit, self.verification))
        result = calibrate_torsion(self.fit, self.verification, **self.options)
        self.assertAlmostEqual(result['branches']['positive']['stiffness_n_m_per_rad'], 18000)
        self.assertAlmostEqual(result['branches']['negative']['stiffness_n_m_per_rad'], 20000)
        self.assertAlmostEqual(result['zero_torque_branch_separation_rad'], .0005, places=13)
        self.assertAlmostEqual(result['zero_torque_branch_center_rad'], .00013, places=13)
        self.assertAlmostEqual(result['verification']['maximum_absolute_error_rad'], 4e-6, places=13)
        self.assertTrue(result['verification']['within_limit'])
        self.assertEqual((self.fit, self.verification), original)
        self.options['verification_limit_rad'] = 3e-6
        failed = calibrate_torsion(self.fit, self.verification, **self.options)
        self.assertFalse(failed['verification']['within_limit'])
        self.assertEqual(result['branches'], failed['branches'])

    def test_rejects_shared_runs_missing_flank_reversed_sign_and_extrapolated_points(self):
        with self.assertRaisesRegex(ValueError, '试次ID'):
            calibrate_torsion(self.fit, samples([1.5, 2.5], 'fit_run'), **self.options)
        with self.assertRaises(ValueError):
            calibrate_torsion(self.fit[:4], self.verification, **self.options)
        wrong_sign = deepcopy(self.verification)
        wrong_sign[0]['torque_n_m'] *= -1
        with self.assertRaisesRegex(ValueError, '扭矩符号'):
            calibrate_torsion(self.fit, wrong_sign, **self.options)
        with self.assertRaisesRegex(ValueError, '拟合范围'):
            calibrate_torsion(self.fit, samples([1.5, 4.5], 'verify_run'), **self.options)
        self.options['fixture_compliance_rad_per_n_m'] = .001
        with self.assertRaisesRegex(ValueError, '柔度必须为正'):
            calibrate_torsion(self.fit, self.verification, **self.options)

    def test_static_model_replay_matches_task_twist_and_aligns_fit_only(self):
        result = calibrate_torsion(self.fit, self.verification, **self.options)
        config = json.loads((ROOT/'examples/dfki_pin_geometry.json').read_text(encoding='utf-8'))
        config['mesh'].update(phases=4, strips=3, elastic_mode='iso')
        model = compare_torsion_model(config, result)
        fit = [row for row in model['samples'] if row['split'] == 'fit']
        self.assertAlmostEqual(sum(row['model_residual_rad'] for row in fit), 0., places=14)
        self.assertEqual(model['evaluated_torque_count'], 14)
        held_out = [row for row in model['samples'] if row['split'] == 'verification']
        loaded_both = [held_out[0], held_out[-1]]
        config['duty_segments'] = [{'label': '静态试验', 'duration_s': 1,
            'output_speed_rad_s': 0, 'output_acceleration_rad_s2': 0,
            'output_torque_n_m': row['torque_n_m'], 'output_wrench': [0, 0, 0, 0, 0, row['torque_n_m']]}
            for row in loaded_both]
        direct = analyze_joint(config)
        backlash = config['precision']['output_backlash_rad']
        for row, case in zip(loaded_both, direct.cases):
            mirror = case['precision']['output_elastic_twist_rad']+np.sign(row['torque_n_m'])*backlash/2
            self.assertAlmostEqual(row['model_angle_rad'], mirror, places=14)
            self.assertAlmostEqual(row['model_tangent_stiffness_n_m_per_rad'],
                                   case['precision']['output_torsional_stiffness_n_m_per_rad'], places=9)
        self.assertNotIn('model_angle_rad', result['samples'][0])

    def test_cli_reproducible_inputs_and_result_manifest(self):
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            report = run(DATA/'fit.csv', DATA/'verification.csv', DATA/'settings.json', directory)
            self.assertTrue(report['verification']['within_limit'])
            self.assertEqual(report['data_kind'], 'synthetic')
            manifest = json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
            for source in manifest['inputs'].values():
                self.assertEqual(source['sha256'], hashlib.sha256((DATA/source['file']).read_bytes()).hexdigest())
            for name, sha in manifest['files'].items():
                self.assertEqual(sha, hashlib.sha256((directory/name).read_bytes()).hexdigest())
