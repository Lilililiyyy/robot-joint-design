"""真实LTCA情景复算、约束方向、缺失判据和危险工况定位验收。"""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from robot_joint import (analyze_joint, analyze_joint_envelope, constraint_margins,
                         export_joint, export_joint_envelope, CouplingConvergenceError)


def small_config():
    root = next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())
    config = json.loads((root/'examples/dfki_pin_geometry.json').read_text(encoding='utf-8'))
    prototype = deepcopy(config['duty_segments'][0])
    config['duty_segments'] = []
    for torque in (2., -2.):
        segment = deepcopy(prototype)
        segment.update(label='正向' if torque > 0 else '反向', duration_s=1., output_torque_n_m=torque,
            output_speed_rad_s=1. if torque > 0 else -1., output_acceleration_rad_s2=0.,
            output_wrench=[0, 0, 0, 0, 0, torque])
        config['duty_segments'].append(segment)
    config['thermal_mode'] = 'single_duty'
    config['load_sharing'].update(sun_radial_stiffness_n_per_m=1e12,
        positive_path_gaps_m=[0, 0, 0], negative_path_gaps_m=[0, 0, 0])
    config['mesh'].update(phases=4, strips=3, elastic_mode='iso')
    config['mesh']['coupling'].update(maximum_iterations=100, force_relative_tolerance=1e-7,
        path_tolerance_m=1e-10, relaxation=.7)
    for name in ('alignment_maps', 'sun_planet_misalignment_m', 'planet_ring_misalignment_m'):
        config['mesh'].pop(name, None)
    config['mesh']['modifications'] = {direction: {'sun_planet': {}, 'planet_ring': {}}
        for direction in ('positive', 'negative')}
    config['design_limits']['maximum_load_share_factor'] = 1.01
    return config


class DesignEnvelopeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = small_config()
        cls.scenarios = [{'id': 'soft_pin', 'description': '仅第一销减径',
            'overrides': {'planet_support': {'pins': [{'diameter_m': .004}, {}, {}]}}}]
        cls.required = ['maximum_load_share_factor', 'contact_pressure_pa', 'planet_pin_static']
        cls.original = deepcopy((cls.config, cls.scenarios, cls.required))
        cls.study = analyze_joint_envelope(cls.config, cls.scenarios, required_checks=cls.required)
        cls.nominal, cls.soft = cls.study['results']

    def test_real_case_passes_nominal_but_exposes_variation_failure(self):
        nominal, soft = self.study['scenarios']
        self.assertTrue(nominal['accepted'])
        self.assertFalse(soft['accepted'])
        self.assertIn('maximum_load_share_factor', soft['failed_checks'])
        self.assertFalse(self.study['all_scenarios_accepted'])
        self.assertEqual(self.study['evaluated_scenario_count'], 2)
        worst = self.study['worst_checks']['maximum_load_share_factor']
        self.assertEqual(worst['scenario_id'], 'soft_pin')
        self.assertLess(worst['relative_margin'], 0)
        for nominal_case, soft_case in zip(self.nominal.cases, self.soft.cases):
            f0, f1 = (np.asarray(c['sharing']['tangential_forces_n']) for c in (nominal_case, soft_case))
            self.assertLess(f1[0], f0[0])
            self.assertTrue(np.all(f1[1:] > f0[1:]))
            self.assertLess(abs(soft_case['sharing']['diagnostics']['output_torque_balance_n_m']), 1e-9)
            self.assertTrue(soft_case['sharing']['coupling']['converged'])

    def test_manual_candidate_and_original_inputs_are_unchanged(self):
        manual = deepcopy(self.config)
        manual['planet_support']['pins'] = [deepcopy(manual['planet_support']['pins']) for _ in range(3)]
        manual['planet_support']['pins'][0]['diameter_m'] = .004
        result = analyze_joint(manual)
        for left, right in zip(result.cases, self.soft.cases):
            np.testing.assert_array_equal(left['sharing']['tangential_forces_n'], right['sharing']['tangential_forces_n'])
            self.assertEqual(left['precision'], right['precision'])
        self.assertEqual((self.config, self.scenarios, self.required), self.original)
        self.assertEqual(self.nominal.inputs, self.config)
        self.assertEqual(result.inputs['design_limits'], self.soft.inputs['design_limits'])

    def test_governing_case_and_component_reproduce_actual_extrema(self):
        checks = constraint_margins(self.soft)
        for name, field in (('maximum_load_share_factor', 'load_share_factor'),):
            check = checks[name]
            case = self.soft.cases[check['governing_location']['case_index']]
            self.assertEqual(case['sharing'][field], check['actual'])
        check = checks['contact_pressure_pa'];where = check['governing_location']
        contact = self.soft.cases[where['case_index']]['contacts'][where['mesh']][where['planet_index']]
        self.assertEqual(contact.summary['max_pressure_pa'], check['actual'])
        check = checks['planet_pin_static'];where = check['governing_location']
        pin = self.soft.planet_support['cases'][where['case_index']]['pins'][where['planet_index']]
        local = pin['checks'][where['stress_kind']]
        self.assertAlmostEqual(check['actual'], local['actual_pa']*local['required_safety_factor']/local['allowable_pa'])
        self.assertIsNone(checks['motor_continuous_screen']['governing_location'])

    def test_margin_directions_zero_limit_and_missing_evaluation(self):
        result = deepcopy(self.nominal)
        values = [('upper', 8., 10., '<=', 'passed'), ('lower', 12., 10., '>=', 'passed'),
                  ('failed', 12., 10., '<=', 'failed'), ('zero', 0., 0., '<=', 'passed'),
                  ('missing', None, None, '<=', 'not_evaluated')]
        result.summary['check_details'] = {name: {'actual': actual, 'threshold': limit,
            'relation': relation, 'status': status, 'scope': '方向验收'}
            for name, actual, limit, relation, status in values}
        checks = constraint_margins(result)
        self.assertEqual(checks['upper']['relative_margin'], .2)
        self.assertEqual(checks['lower']['relative_margin'], .2)
        self.assertEqual(checks['failed']['relative_margin'], -.2)
        self.assertEqual(checks['zero']['margin'], 0)
        self.assertIsNone(checks['zero']['relative_margin'])
        self.assertIsNone(checks['missing']['margin'])

    def test_required_unconfigured_or_unevaluated_checks_block_acceptance(self):
        config = deepcopy(self.config)
        config['design_limits'].pop('contact_pressure_pa')
        study = analyze_joint_envelope(config, [{'id': 'same', 'overrides': {
            'load_sharing': {'positive_path_gaps_m': [0, 0, 0]}}}],
            required_checks=['contact_pressure_pa', 'dc_bus_minimum_voltage'])
        self.assertTrue(study['scenarios'][0]['summary']['configured_checks_pass'])
        self.assertFalse(study['all_scenarios_accepted'])
        for row in study['scenarios']:
            self.assertEqual(row['required_checks_missing'], ['contact_pressure_pa', 'dc_bus_minimum_voltage'])
            self.assertIsNone(row['checks']['contact_pressure_pa']['relative_margin'])
            self.assertIsNone(row['checks']['dc_bus_minimum_voltage']['relative_margin'])

    def test_path_changes_locate_the_governing_loaded_flank(self):
        study = analyze_joint_envelope(self.config, [{'id': 'positive_gap', 'overrides': {
            'load_sharing': {'positive_path_gaps_m': [0, 8e-6, 8e-6]}}}], required_checks=self.required)
        check = study['worst_checks']['maximum_load_share_factor']
        self.assertEqual(check['scenario_id'], 'positive_gap')
        self.assertEqual(check['governing_location']['case_index'], 0)
        self.assertAlmostEqual(check['actual'], 3.)
        self.assertLess(check['relative_margin'], 0)

    def test_explicit_stiffness_support_and_misalignment_changes_are_applied(self):
        config = deepcopy(self.config)
        config['load_sharing']['pin_stiffness_source'] = 'specified'
        config['load_sharing']['pin_stiffness_n_per_m'] = self.nominal.pin_stiffness['adopted_pin_stiffness_n_per_m']
        support = (.5*np.asarray(config['support']['stiffness_matrix'])).tolist()
        scenarios = [{'id': 'support_changes', 'overrides': {
            'load_sharing': {'pin_stiffness_n_per_m': [7.1e7, 2.97e8, 2.97e8],
                             'sun_radial_stiffness_n_per_m': 1e7},
            'support': {'stiffness_matrix': support},
            'mesh': {'sun_planet_misalignment_m': [1e-6, 0., 0.]}}}]
        study = analyze_joint_envelope(config, scenarios, required_checks=self.required)
        result = study['results'][1]
        self.assertEqual(result.inputs['support']['stiffness_matrix'], support)
        self.assertEqual(result.pin_stiffness['adopted_pin_stiffness_n_per_m'], [7.1e7, 2.97e8, 2.97e8])
        self.assertLess(result.summary['minimum_output_torsional_stiffness_n_m_per_rad'],
                        study['results'][0].summary['minimum_output_torsional_stiffness_n_m_per_rad'])
        with self.assertRaises(ValueError):
            analyze_joint_envelope(config, self.scenarios, required_checks=self.required)

    def test_invalid_or_acceptance_changing_scenarios_are_rejected(self):
        invalid = [{'id': 'nominal', 'overrides': {}}, {'id': '../escape', 'overrides': {}},
            {'id': 'limits', 'overrides': {'design_limits': {'maximum_load_share_factor': 10}}},
            {'id': 'stage', 'overrides': {'stage': {'sun_teeth': 18}}},
            {'id': 'allowable', 'overrides': {'planet_support': {'pins': {'allowable_bending_stress_pa': 1e12}}}},
            {'id': 'ignored', 'overrides': {'load_sharing': {'pin_stiffness_n_per_m': 1e6}}}]
        for scenario in invalid:
            with self.subTest(scenario=scenario), self.assertRaises(ValueError):
                analyze_joint_envelope(self.config, [scenario], required_checks=self.required)
        for scenarios, required in (([], self.required), (self.scenarios*2, self.required),
                                    (self.scenarios, []), (self.scenarios, ['typo'])):
            with self.assertRaises(ValueError):
                analyze_joint_envelope(self.config, scenarios, required_checks=required)

    def test_nonconvergence_does_not_return_an_envelope(self):
        config = deepcopy(self.config)
        config['mesh']['coupling']['maximum_iterations'] = 1
        with self.assertRaisesRegex(CouplingConvergenceError, 'nominal'):
            analyze_joint_envelope(config, self.scenarios, required_checks=self.required)

    def test_reports_inputs_manifest_and_normal_export_margins(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            folder = export_joint_envelope(self.study, root/'study')
            manifest = json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
            for name, digest in manifest['files'].items():
                self.assertEqual(hashlib.sha256((folder/name).read_bytes()).hexdigest(), digest)
            study = json.loads((folder/'study.json').read_text(encoding='utf-8'))
            self.assertNotIn('results', study)
            self.assertEqual(json.loads((folder/'soft_pin_input.json').read_text(encoding='utf-8')), self.soft.inputs)
            self.assertTrue((folder/'report.en.md').is_file())
            single = export_joint(self.nominal, root/'single')
            margins = json.loads((single/'constraint_margins.json').read_text(encoding='utf-8'))
            self.assertEqual(margins['maximum_load_share_factor']['status'], 'passed')


if __name__ == '__main__':
    unittest.main()
