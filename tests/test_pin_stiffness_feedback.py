"""行星销几何刚度进入真实分载/LTCA的集成验收。

少量相位与条带只缩小真实求解规模，不替换齿面响应。几何推导刚度
与手工回填既有显式刚度路径逐项对照，并复查最终反力、来源与导出。
"""
from copy import deepcopy
import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from robot_joint.analysis import analyze_joint
from robot_joint.gears import coupled_planet_contacts, CouplingConvergenceError
from robot_joint.io import export_joint
from robot_joint.planetary import PlanetaryStage
from robot_joint.planet_support import pin_response, resolve_pin_stiffness


def _example_config():
    """通过公开示例标记定位工程，不依赖启动目录或固定父目录层数。"""
    for directory in Path(__file__).resolve().parents:
        candidate = directory / 'examples' / 'dfki_with_screens.json'
        if candidate.is_file():
            return json.loads(candidate.read_text(encoding='utf-8'))
    raise FileNotFoundError('未找到行星销反馈验收所需示例')


def _small_config():
    """构造对称双向工况，使销柔度变化成为唯一的不均载来源。"""
    config = _example_config()
    prototype = deepcopy(config['duty_segments'][0])
    config['duty_segments'] = []
    for torque in (2.0, -2.0):
        segment = deepcopy(prototype)
        segment.update(label='正向' if torque > 0 else '反向', duration_s=1.0,
                       output_torque_n_m=torque, output_speed_rad_s=1.0 if torque > 0 else -1.0,
                       output_acceleration_rad_s2=0.0, output_wrench=[0, 0, 0, 0, 0, torque])
        config['duty_segments'].append(segment)
    config['thermal_mode'] = 'single_duty'
    config['load_sharing'].update(pin_stiffness_source='pin_geometry',
                                 sun_radial_stiffness_n_per_m=1e12,
                                 positive_path_gaps_m=[0, 0, 0], negative_path_gaps_m=[0, 0, 0])
    config['load_sharing'].pop('pin_stiffness_n_per_m', None)
    config['mesh'].update(phases=4, strips=3, elastic_mode='iso')
    config['mesh']['coupling'].update(maximum_iterations=100, force_relative_tolerance=1e-7,
                                     path_tolerance_m=1e-10, relaxation=.7)
    config['mesh'].pop('alignment_maps', None)
    for name in ('sun_planet_misalignment_m', 'planet_ring_misalignment_m'):
        config['mesh'].pop(name, None)
    for direction in ('positive', 'negative'):
        config['mesh']['modifications'][direction] = {'sun_planet': {}, 'planet_ring': {}}
    return config


class PinStiffnessResolutionTests(unittest.TestCase):
    def setUp(self):
        self.pin = _example_config()['planet_support']['pins']

    def test_shared_geometry_uses_work_conjugate_bending_and_shear_compliance(self):
        sharing = {'pin_stiffness_source': 'pin_geometry'}
        before = deepcopy((sharing, self.pin))
        result = resolve_pin_stiffness(3, sharing, pins=self.pin)
        direct = pin_response([0, 0], **self.pin)
        bending = direct['load_weighted_bending_compliance_m_per_n']
        shear = direct['load_weighted_shear_compliance_m_per_n']
        expected = 1 / (bending + shear)
        np.testing.assert_allclose(result['adopted_pin_stiffness_n_per_m'], [expected]*3, rtol=1e-14)
        np.testing.assert_allclose(result['path_compliance_contribution_m_per_n'], [4/expected]*3, rtol=1e-14)
        self.assertEqual(result['source'], 'pin_geometry')
        self.assertFalse(result['specified_input_used'])
        self.assertIsNone(result['specified_input_n_per_m'])
        self.assertEqual((sharing, self.pin), before)
        for index, record in enumerate(result['planets']):
            self.assertEqual(record['planet_index'], index)
            self.assertAlmostEqual(record['load_weighted_bending_compliance_m_per_n'], bending)
            self.assertAlmostEqual(record['load_weighted_shear_compliance_m_per_n'], shear)
            self.assertEqual(record['source_parameters']['diameter_m'], self.pin['diameter_m'])
            self.assertEqual(record['short_pin_review_required'], direct['diagnostics']['short_pin_review_required'])

    def test_per_pin_geometry_and_ignored_explicit_input_preserve_provenance(self):
        pins = [deepcopy(self.pin) for _ in range(3)]
        pins[0]['diameter_m'] *= .7
        pins[2]['support_span_m'] *= 1.2
        explicit = [1e5, 2e5, 3e5]
        sharing = {'pin_stiffness_source': 'pin_geometry', 'pin_stiffness_n_per_m': explicit}
        before = deepcopy((pins, sharing))
        result = resolve_pin_stiffness(3, sharing, pins=pins)
        expected = [pin_response([0, 0], **pin)['equivalent_pin_stiffness_n_per_m'] for pin in pins]
        np.testing.assert_allclose(result['adopted_pin_stiffness_n_per_m'], expected, rtol=1e-14)
        self.assertEqual(result['specified_input_n_per_m'], explicit)
        self.assertFalse(result['specified_input_used'])
        self.assertEqual((pins, sharing), before)

    def test_default_and_explicit_specified_mode_preserve_scalar_and_list_contracts(self):
        for supplied in (7e8, [7e8, 8e8, 9e8]):
            with self.subTest(supplied=supplied):
                expected = [supplied]*3 if np.isscalar(supplied) else supplied
                default = resolve_pin_stiffness(3, {'pin_stiffness_n_per_m': supplied})
                explicit = resolve_pin_stiffness(3, {'pin_stiffness_source': 'specified',
                                                    'pin_stiffness_n_per_m': supplied})
                self.assertEqual(default, explicit)
                self.assertEqual(default['source'], 'specified')
                self.assertTrue(default['specified_input_used'])
                self.assertEqual(default['specified_input_n_per_m'], supplied)
                np.testing.assert_allclose(default['adopted_pin_stiffness_n_per_m'], expected)

    def test_invalid_mode_or_incomplete_pin_configuration_is_explicitly_rejected(self):
        variants = [({'pin_stiffness_source': 'automatic'}, self.pin),
                    ({'pin_stiffness_source': 'pin_geometry'}, None),
                    ({'pin_stiffness_source': 'pin_geometry'}, [self.pin, self.pin]),
                    ({'pin_stiffness_source': 'pin_geometry'}, dict(self.pin, diameter_m=0)),
                    ({'pin_stiffness_source': 'specified'}, self.pin)]
        incomplete = dict(self.pin)
        incomplete.pop('loaded_width_m')
        variants.append(({'pin_stiffness_source': 'pin_geometry'}, incomplete))
        for sharing, pins in variants:
            with self.subTest(sharing=sharing, pins=pins), self.assertRaises(ValueError):
                resolve_pin_stiffness(3, sharing, pins=pins)

    def test_nonfinite_geometry_stiffness_cannot_enter_the_system(self):
        # 极端输入只检验数值域保护，绝非实际工程推荐尺寸或材料。
        pin = dict(self.pin, diameter_m=1.0, support_span_m=1e-8,
                   loaded_width_m=1e-8, young_modulus_pa=1e300)
        with self.assertRaises(ValueError):
            resolve_pin_stiffness(3, {'pin_stiffness_source': 'pin_geometry'}, pins=pin)

    def test_low_level_contact_entry_rejects_unresolved_geometry_or_unknown_source(self):
        config = _small_config()
        stage = PlanetaryStage(**config['stage'])
        for source in ('pin_geometry', 'unknown'):
            sharing = dict(config['load_sharing'], pin_stiffness_source=source,
                           pin_stiffness_n_per_m=8e8)
            with self.subTest(source=source), self.assertRaises(ValueError) as error:
                coupled_planet_contacts(stage, 2.0, sharing, config['mesh'])
            self.assertRegex(str(error.exception), '[\u4e00-\u9fff]')


class PinStiffnessFeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.symmetric_config = _small_config()
        cls.symmetric_before = deepcopy(cls.symmetric_config)
        cls.symmetric = analyze_joint(cls.symmetric_config)

        cls.soft_config = _small_config()
        pins = [deepcopy(cls.soft_config['planet_support']['pins']) for _ in range(3)]
        pins[0]['diameter_m'] = .004
        cls.soft_config['planet_support']['pins'] = pins
        # 故意保留不同的旧显式数值，证明几何模式未继续采用它们。
        cls.soft_config['load_sharing']['pin_stiffness_n_per_m'] = [1e5, 2e5, 3e5]
        cls.soft_before = deepcopy(cls.soft_config)
        cls.soft = analyze_joint(cls.soft_config)

        cls.manual_config = deepcopy(cls.soft_config)
        cls.manual_config['load_sharing']['pin_stiffness_source'] = 'specified'
        cls.manual_config['load_sharing']['pin_stiffness_n_per_m'] = [
            pin_response([0, 0], **pin)['equivalent_pin_stiffness_n_per_m'] for pin in pins]
        cls.manual_before = deepcopy(cls.manual_config)
        cls.manual = analyze_joint(cls.manual_config)

        cls.default_config = deepcopy(cls.manual_config)
        cls.default_config['load_sharing'].pop('pin_stiffness_source')
        cls.default_before = deepcopy(cls.default_config)
        cls.default = analyze_joint(cls.default_config)

    def test_real_symmetric_double_direction_loads_and_contact_arrays(self):
        result = self.symmetric
        lever = 2*result.geometry['carrier_radius_m']
        expected = 2.0 / lever / 3
        for case in result.cases:
            np.testing.assert_allclose(case['sharing']['tangential_forces_n'], [expected]*3, rtol=2e-7)
            self.assertAlmostEqual(case['sharing']['load_share_factor'], 1.0, places=6)
            for group in case['contacts'].values():
                self.assertEqual(len(group), 3)
                for analysis in group:
                    self.assertEqual(len(analysis.rotations), 4)
                    self.assertTrue(analysis.summary['all_converged'])
                    self.assertGreater(sum(rotation.solution.forces_n.size for rotation in analysis.rotations), 0)
        np.testing.assert_allclose(result.cases[0]['sharing']['carrier_pin_forces_xy_n'],
                                   -np.asarray(result.cases[1]['sharing']['carrier_pin_forces_xy_n']), atol=1e-8)

    def test_geometry_feedback_retains_zero_load_convention_with_real_contact_solver(self):
        config = _small_config()
        config['duty_segments'] = [config['duty_segments'][0]]
        config['duty_segments'][0].update(output_torque_n_m=0.0, output_speed_rad_s=0.0,
                                         output_wrench=[0, 0, 0, 0, 0, 0])
        # 全零载下均载系数和滚动寿命未定义，不沿用原示例的对应性能限值。
        config.pop('design_limits', None)
        before = deepcopy(config)
        result = analyze_joint(config)
        sharing = result.cases[0]['sharing']
        self.assertTrue(sharing['coupling']['converged'])
        self.assertEqual(sharing['output_elastic_twist_rad'], 0.0)
        self.assertIsNone(sharing['load_share_factor'])
        np.testing.assert_array_equal(sharing['carrier_pin_forces_xy_n'], np.zeros((3, 2)))
        np.testing.assert_allclose(sharing['pin_stiffness_n_per_m'],
                                   result.pin_stiffness['adopted_pin_stiffness_n_per_m'])
        self.assertEqual(result.inputs, before)
        self.assertEqual(config, before)
        for group in result.cases[0]['contacts'].values():
            for contact in group:
                self.assertTrue(contact.summary['all_converged'])
                self.assertEqual(contact.summary['max_pressure_pa'], 0.0)

    def test_softening_one_pin_reduces_its_load_and_increases_other_loads(self):
        base_k = self.symmetric.pin_stiffness['adopted_pin_stiffness_n_per_m']
        soft_k = self.soft.pin_stiffness['adopted_pin_stiffness_n_per_m']
        self.assertLess(soft_k[0], base_k[0])
        np.testing.assert_allclose(soft_k[1:], base_k[1:], rtol=1e-14)
        for base, soft in zip(self.symmetric.cases, self.soft.cases):
            f0 = np.asarray(base['sharing']['tangential_forces_n'])
            f1 = np.asarray(soft['sharing']['tangential_forces_n'])
            self.assertLess(f1[0], f0[0]*.95)
            self.assertTrue(np.all(f1[1:] > f0[1:]))
            self.assertGreater(soft['sharing']['load_share_factor'], base['sharing']['load_share_factor'])

    def test_geometry_feedback_with_real_path_opening_and_reentry(self):
        config = _small_config()
        config.pop('design_limits', None)
        config['load_sharing']['positive_path_gaps_m'] = [0, 8e-6, 8e-6]
        for segment, torque in zip(config['duty_segments'], (.01, 40.0)):
            segment.update(output_torque_n_m=torque, output_speed_rad_s=1.0,
                           output_wrench=[0, 0, 0, 0, 0, torque])
        result = analyze_joint(config)
        low, high = (case['sharing'] for case in result.cases)
        self.assertEqual(np.count_nonzero(low['active_mask']), 1)
        self.assertEqual(np.count_nonzero(high['active_mask']), 3)
        np.testing.assert_allclose(low['tangential_forces_n'][1:], [0, 0], atol=1e-10)
        self.assertTrue(np.all(np.asarray(high['tangential_forces_n']) > 0))
        for sharing in (low, high):
            self.assertTrue(sharing['coupling']['converged'])
            self.assertLessEqual(sharing['coupling']['path_residual_m'],
                                 config['mesh']['coupling']['path_tolerance_m'])
            self.assertLess(abs(sharing['diagnostics']['output_torque_balance_n_m']), 1e-9)

    def test_geometry_feedback_matches_manual_filling_of_existing_specified_path(self):
        for geometry_case, manual_case in zip(self.soft.cases, self.manual.cases):
            for key in ('tangential_forces_n', 'normal_forces_n', 'carrier_pin_forces_xy_n',
                        'compliance_m_per_n', 'effective_path_gaps_m'):
                np.testing.assert_allclose(geometry_case['sharing'][key], manual_case['sharing'][key], rtol=1e-12, atol=1e-14)
            self.assertAlmostEqual(geometry_case['sharing']['output_torsional_stiffness_n_m_per_rad'],
                                   manual_case['sharing']['output_torsional_stiffness_n_m_per_rad'], places=10)
            for name in ('sun_planet', 'planet_ring'):
                for computed, manual in zip(geometry_case['contacts'][name], manual_case['contacts'][name]):
                    np.testing.assert_allclose(computed.normal_approach_m, manual.normal_approach_m, rtol=1e-12)
                    self.assertAlmostEqual(computed.summary['max_pressure_pa'], manual.summary['max_pressure_pa'], places=7)

    def test_converged_torque_path_and_contact_force_residuals(self):
        for result in (self.symmetric, self.soft):
            self.assertTrue(result.pin_stiffness['coupling']['all_converged'])
            self.assertEqual(len(result.pin_stiffness['coupling']['cases']), 2)
            self.assertTrue(result.summary['all_couplings_converged'])
            adopted = result.pin_stiffness['adopted_pin_stiffness_n_per_m']
            for case, segment in zip(result.cases, result.inputs['duty_segments']):
                sharing = case['sharing']
                self.assertEqual(sharing['pin_stiffness_source'], 'pin_geometry')
                np.testing.assert_allclose(sharing['pin_stiffness_n_per_m'], adopted, rtol=1e-14)
                diagnostics = sharing['diagnostics']
                self.assertLess(abs(diagnostics['output_torque_balance_n_m']), 1e-9)
                self.assertLess(abs(diagnostics['external_torque_balance_n_m']), 1e-9)
                np.testing.assert_allclose(diagnostics['external_force_balance_xy_n'], [0, 0], atol=1e-8)
                tolerance = result.inputs['mesh']['coupling']['path_tolerance_m']
                self.assertLessEqual(sharing['coupling']['path_residual_m'], tolerance)
                approach = np.asarray(sharing['compliance_m_per_n']) @ sharing['tangential_forces_n']
                approach += sharing['effective_path_gaps_m']
                np.testing.assert_allclose(approach, sharing['path_approach_m'], rtol=0, atol=tolerance*1.1)
                torque = 2*result.geometry['carrier_radius_m']*float(np.sum(sharing['tangential_forces_n']))
                self.assertAlmostEqual(torque, abs(segment['output_torque_n_m']), places=9)
                for group in case['contacts'].values():
                    for analysis in group:
                        self.assertLess(analysis.summary['max_force_residual_n'], 1e-7)

    def test_final_pin_and_bearing_checks_use_converged_reactions_and_adopted_stiffness(self):
        self.assertTrue(self.soft.planet_support['summary']['system_stiffness_was_updated'])
        adopted = self.soft.pin_stiffness['adopted_pin_stiffness_n_per_m']
        for index, pin_summary in enumerate(self.soft.planet_support['pins']):
            self.assertAlmostEqual(pin_summary['specified_pin_stiffness_n_per_m'], adopted[index])
            self.assertAlmostEqual(pin_summary['stiffness_ratio_to_system_input'], 1.0, places=12)
        for case, assessed in zip(self.soft.cases, self.soft.planet_support['cases']):
            for index, actual in enumerate(assessed['pins']):
                force = case['sharing']['carrier_pin_forces_xy_n'][index]
                expected = pin_response(force, **self.soft_config['planet_support']['pins'][index])
                np.testing.assert_allclose(actual['force_xy_n'], force, rtol=1e-14)
                self.assertAlmostEqual(actual['center_deflection_m'], expected['center_deflection_m'], places=15)
                self.assertAlmostEqual(actual['equivalent_pin_stiffness_n_per_m'], adopted[index], places=6)
        for index, bearing in enumerate(self.soft.planet_support['planet_bearings']):
            expected_loads = [np.linalg.norm(case['sharing']['carrier_pin_forces_xy_n'][index])
                              for case in self.soft.cases]
            np.testing.assert_allclose(bearing['equivalent_dynamic_load_n'], expected_loads[0], rtol=1e-6)

    def test_default_specified_mode_and_all_original_inputs_are_preserved(self):
        for config, original, result in ((self.symmetric_config, self.symmetric_before, self.symmetric),
                                         (self.soft_config, self.soft_before, self.soft),
                                         (self.manual_config, self.manual_before, self.manual),
                                         (self.default_config, self.default_before, self.default)):
            self.assertEqual(config, original)
            self.assertEqual(result.inputs, original)
        self.assertEqual(self.soft.pin_stiffness['specified_input_n_per_m'], [1e5, 2e5, 3e5])
        self.assertFalse(self.soft.pin_stiffness['specified_input_used'])
        for result in (self.manual, self.default):
            self.assertEqual(result.pin_stiffness['source'], 'specified')
            self.assertTrue(result.pin_stiffness['specified_input_used'])
            self.assertFalse(result.planet_support['summary']['system_stiffness_was_updated'])
        for manual_case, default_case in zip(self.manual.cases, self.default.cases):
            np.testing.assert_allclose(manual_case['sharing']['tangential_forces_n'],
                                       default_case['sharing']['tangential_forces_n'], rtol=1e-14)

    def test_geometry_mode_requires_complete_planet_support_configuration(self):
        for missing in ('planet_support', 'pins', 'bearing_ratings'):
            config = deepcopy(self.symmetric_config)
            if missing == 'planet_support':
                config.pop(missing)
            else:
                config['planet_support'].pop(missing)
            with self.subTest(missing=missing), self.assertRaises(ValueError):
                analyze_joint(config)

    def test_export_records_adopted_values_and_source_in_manifest_and_bom(self):
        with tempfile.TemporaryDirectory() as directory:
            root = export_joint(self.soft, directory, plots=False)
            provenance = json.loads((root/'pin_stiffness.json').read_text(encoding='utf-8'))
            self.assertEqual(provenance['source'], 'pin_geometry')
            self.assertFalse(provenance['specified_input_used'])
            np.testing.assert_allclose(provenance['adopted_pin_stiffness_n_per_m'],
                                       self.soft.pin_stiffness['adopted_pin_stiffness_n_per_m'], rtol=1e-14)
            self.assertEqual(json.loads((root/'input.json').read_text(encoding='utf-8')), self.soft_before)
            manifest = json.loads((root/'manifest.json').read_text(encoding='utf-8'))
            self.assertEqual(manifest['files']['pin_stiffness.json'],
                             hashlib.sha256((root/'pin_stiffness.json').read_bytes()).hexdigest())
            with (root/'bom.csv').open(encoding='utf-8-sig', newline='') as stream:
                pin = next(row for row in csv.DictReader(stream) if row['item_id'] == 'planet_pin')
            specification = json.loads(pin['specification'])
            np.testing.assert_allclose(specification['tangential_stiffness_n_per_m'],
                                       provenance['adopted_pin_stiffness_n_per_m'], rtol=1e-14)
            self.assertFalse(np.allclose(specification['tangential_stiffness_n_per_m'],
                                        self.soft_before['load_sharing']['pin_stiffness_n_per_m']))

    def test_exported_postprocessing_uses_adopted_values_and_rejects_mixed_or_missing_records(self):
        from examples.check_planet_support import load_exported_duty, run
        assumptions = {'candidates': [{'id': 'actual_pin', **self.soft_config['planet_support']}]}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            export_joint(self.soft, root/'joint', plots=False)
            base, cases, provenance = load_exported_duty(root/'joint')
            run(base, cases, assumptions, provenance, root/'support')
            support = json.loads((root/'support/actual_pin.json').read_text(encoding='utf-8'))
            for pin in support['pins']:
                self.assertAlmostEqual(pin['stiffness_ratio_to_system_input'], 1.0, places=12)
            missing = deepcopy(cases)
            missing[1]['sharing'].pop('pin_stiffness_n_per_m')
            with self.assertRaises(ValueError):
                run(base, missing, assumptions, provenance, root/'missing')
            mixed = deepcopy(cases)
            mixed[1]['sharing']['pin_stiffness_n_per_m'][0] *= .5
            with self.assertRaises(ValueError):
                run(base, mixed, assumptions, provenance, root/'mixed')

    def test_candidate_comparison_keeps_geometry_base_and_explicitly_softens_adopted_values(self):
        from examples.pendulum_case import candidate_configs
        base, errors, softer = candidate_configs(self.symmetric_config)
        self.assertEqual(base['load_sharing']['pin_stiffness_source'], 'pin_geometry')
        self.assertEqual(errors['load_sharing']['pin_stiffness_source'], 'pin_geometry')
        self.assertEqual(softer['load_sharing']['pin_stiffness_source'], 'specified')
        np.testing.assert_allclose(softer['load_sharing']['pin_stiffness_n_per_m'],
            .3*np.asarray(self.symmetric.pin_stiffness['adopted_pin_stiffness_n_per_m']))

    def test_reproducible_comparison_recalculates_diameter_variants_and_saves_bound_inputs(self):
        from examples.compare_pin_stiffness import run
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            summary = run(self.symmetric_config, root)
            rows = {row['scenario']: row for row in summary['records']}
            self.assertTrue(summary['manual_equivalence']['all_response_arrays_identical'])
            self.assertTrue(all(row['all_couplings_converged'] for row in rows.values()))
            self.assertLess(rows['all_smaller']['minimum_output_stiffness_n_m_per_rad'],
                            rows['geometry_reference']['minimum_output_stiffness_n_m_per_rad'])
            with np.load(root/'comparison.npz') as saved:
                base = saved['geometry_reference_forces_n']
                soft = saved['first_smaller_forces_n']
                self.assertTrue(np.all(soft[:, 0] < base[:, 0]))
                self.assertTrue(np.all(soft[:, 1:] > base[:, 1:]))
            manifest = json.loads((root/'manifest.json').read_text(encoding='utf-8'))
            for name, digest in manifest['files'].items():
                self.assertEqual(hashlib.sha256((root/name).read_bytes()).hexdigest(), digest)
            self.assertEqual(json.loads((root/'source_input.json').read_text(encoding='utf-8')),
                             self.symmetric_before)
            original_csv = (root/'sharing.csv').read_bytes()
            repeated = run(self.symmetric_config, root)
            self.assertEqual(repeated, summary)
            self.assertEqual((root/'sharing.csv').read_bytes(), original_csv)

    def test_comparison_stops_when_real_coordination_does_not_converge(self):
        from examples.compare_pin_stiffness import run
        config = deepcopy(self.symmetric_config)
        config['mesh']['coupling'].update(maximum_iterations=1,
            force_relative_tolerance=1e-14, path_tolerance_m=1e-16)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(CouplingConvergenceError):
                run(config, directory)
            self.assertFalse((Path(directory)/'report.md').exists())
            self.assertFalse((Path(directory)/'summary.json').exists())
            previous = {'report.md': '上一次已完成报告', 'summary.json': '{"previous": true}',
                        'source_input.json': '{"previous_input": true}'}
            for name, content in previous.items():
                (Path(directory)/name).write_text(content, encoding='utf-8')
            with self.assertRaises(CouplingConvergenceError):
                run(config, directory)
            for name, content in previous.items():
                self.assertEqual((Path(directory)/name).read_text(encoding='utf-8'), content)


if __name__ == '__main__':
    unittest.main()
