"""独立复核斑点材料映射、相位身份及内部相位的压力积分。"""
from copy import deepcopy
import math
import unittest

import numpy as np

from gear_contact import Gear, GearPair, analyze_pair
from robot_joint.contact_pattern import build_contact_pattern, interior_snapshot


class ContactPatternIndependentReviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = []
        for internal, pinion_teeth, wheel_teeth in ((False, 20, 40), (True, 40, 100)):
            pair = GearPair(Gear(pinion_teeth, .0005, .005),
                            Gear(wheel_teeth, .0005, .005, internal=internal))
            cls.results.append(analyze_pair(pair, torque_n_m=.15, phases=40, strips=7,
                elastic_mode='iso', local_contact=False,
                iso_settings={'nominal_pinion_torque_n_m': .15}))

    @staticmethod
    def integral(pattern, pair, member):
        """使用渐开线弧长的实际面积，不能直接按roll坐标积分压力。"""
        data = pattern.arrays
        arc_width = np.diff(data['snapshot_profile_roll_edges_m']**2 /
                            (2*getattr(pair, member).base_radius_m))
        face_width = np.diff(data['face_edges_m'])
        return float(np.sum(data['instant_pressure_pa']*arc_width[:, None]*face_width[None, :]))

    def test_external_and_internal_wheel_path_signs(self):
        for result in self.results:
            pair = result.pair
            span = pair.center_distance_m*math.sin(pair.working_pressure_angle_rad)
            low, high = pair.contact_roll_bounds_m
            expected = sorted((span+low, span+high) if pair.wheel.internal else
                              (span-low, span-high))
            pattern = build_contact_pattern(result, member='wheel', profile_points=1001)
            np.testing.assert_allclose(pattern.summary['theoretical_contact_roll_bounds_m'], expected,
                                       rtol=1e-13, atol=1e-15)
            data = pattern.arrays
            calculated = span+data['roll_pinion_m'] if pair.wheel.internal else span-data['roll_pinion_m']
            np.testing.assert_allclose(data['roll_wheel_m'], calculated, rtol=1e-13, atol=1e-15)

    def test_interior_snapshot_integrates_the_selected_pair_force_on_both_members(self):
        # 第5相位位于有效齿面内部，避开无限局部Hertz带遭齿顶裁剪的已知边缘问题。
        for result in self.results:
            for member in ('pinion', 'wheel'):
                pattern = build_contact_pattern(result, member=member, profile_points=5001,
                                                snapshot_index=5)
                data = pattern.arrays
                selected = ((data['phase_index'] == 5) &
                            (data['tooth_pair_index'] == pattern.summary['snapshot_tooth_pair_index']))
                expected = float(np.sum(data['force_n'][selected]))
                self.assertLess(abs(self.integral(pattern, result.pair, member)-expected)/expected, .003)

    def test_raw_force_total_and_pressure_peak_survive_display_mapping(self):
        for result in self.results:
            pattern = build_contact_pattern(result, profile_points=1201)
            data = pattern.arrays
            for phase_index in range(len(result.phase)):
                self.assertAlmostEqual(np.sum(data['force_n'][data['phase_index'] == phase_index]),
                                       result.summary['normal_force_n'], places=10)
            self.assertLessEqual(pattern.summary['map_peak_pressure_pa'],
                                 pattern.summary['source_peak_pressure_pa']*(1+1e-14))
            self.assertFalse(pattern.summary['physical_validation'])
            self.assertIn('非二维', pattern.summary['scope'])

    def test_uniform_full_cycle_may_wrap_without_changing_material_samples(self):
        result = self.results[0]
        moved = deepcopy(result)
        order = np.roll(np.arange(len(result.phase)), 13)
        moved.phase = result.phase[order]
        moved.rotations = [result.rotations[index] for index in order]
        a = build_contact_pattern(result, profile_points=1001)
        b = build_contact_pattern(moved, profile_points=1001)
        np.testing.assert_allclose(a.arrays['sweep_pressure_pa'], b.arrays['sweep_pressure_pa'],
                                   rtol=0, atol=0)

    def test_incomplete_phase_cycle_is_rejected(self):
        result = deepcopy(self.results[0])
        result.phase = result.phase[:-1]
        result.rotations = result.rotations[:-1]
        with self.assertRaisesRegex(ValueError, '完整'):
            build_contact_pattern(result)

    def test_zero_load_preserves_zero_pressure_and_half_width(self):
        pair = self.results[0].pair
        result = analyze_pair(pair, torque_n_m=0., phases=8, strips=5,
            elastic_mode='iso', local_contact=False, iso_settings={'nominal_pinion_torque_n_m': 0.})
        pattern = build_contact_pattern(result, member='wheel', profile_points=201)
        for name in ('sweep_pressure_pa', 'instant_pressure_pa', 'half_width_m', 'force_n'):
            np.testing.assert_array_equal(pattern.arrays[name], 0.)

    def test_default_peak_wheel_snapshot_reports_half_band_tip_clipping(self):
        # 峰压原中心位于大轮齿顶；解析有限齿面裁剪须诚实保留约50%的原带力。
        for result in self.results:
            pattern = build_contact_pattern(result, member='wheel', profile_points=1001)
            summary = pattern.summary
            ratio = summary['snapshot_analytical_clipped_force_n']/summary['snapshot_selected_pair_force_n']
            self.assertGreater(ratio, .49)
            self.assertLess(ratio, .51)
            self.assertLess(summary['snapshot_grid_force_relative_error'], .003)
            self.assertAlmostEqual(self.integral(pattern, result.pair, 'wheel'),
                                   summary['snapshot_grid_integrated_force_n'], places=10)

    def test_low_load_adaptive_snapshot_resolves_the_narrow_hertz_band(self):
        pair = self.results[0].pair
        result = analyze_pair(pair, torque_n_m=1e-6, phases=12, strips=5,
            elastic_mode='iso', local_contact=False, iso_settings={'nominal_pinion_torque_n_m': 1e-6})
        phase, tooth = interior_snapshot(result)
        for member in ('pinion', 'wheel'):
            pattern = build_contact_pattern(result, member=member, profile_points=201,
                                            snapshot_index=phase, snapshot_tooth_pair_index=tooth)
            summary = pattern.summary
            expected = summary['snapshot_selected_pair_force_n']
            self.assertLess(abs(self.integral(pattern, pair, member)-expected)/expected, .003)
            self.assertLess(summary['snapshot_grid_force_relative_error'], .003)

    def test_external_tooth_dependent_callbacks_cannot_be_folded_as_identical_teeth(self):
        for field in ('external_boundary_gap', 'external_compliance_provider'):
            result = deepcopy(self.results[0])
            result.inputs[field] = True
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, '回调'):
                build_contact_pattern(result)


if __name__ == '__main__':
    unittest.main()
