"""有限Hertz带的弧长映射、力守恒、周期覆盖与明确模型边界。"""
from copy import deepcopy
import math
import tempfile
from pathlib import Path
import json
import unittest
import numpy as np
from gear_contact import Gear,GearPair,analyze_pair,SurfaceModification
from robot_joint.contact_pattern import build_contact_pattern,export_contact_pattern


def solve(*,phases=64,torque=5,internal=False,crown=0):
    pinion=Gear(40 if internal else 20,.002,.01)
    wheel=Gear(100 if internal else 40,.002,.01,internal=internal)
    return analyze_pair(GearPair(pinion,wheel),torque_n_m=torque,phases=phases,strips=7,
        elastic_mode='iso',local_contact=False,iso_settings={'nominal_pinion_torque_n_m':torque},
        modification_pinion=SurfaceModification(lead_crown_m=crown))


class ContactPatternTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result=solve()
        cls.pattern=build_contact_pattern(cls.result,profile_points=1001)

    def test_hertz_force_is_recovered_from_pressure_and_area(self):
        a=self.pattern.arrays
        recovered=math.pi/2*a['pressure_pa']*a['half_width_m']*a['strip_width_m']
        np.testing.assert_allclose(recovered,a['force_n'],rtol=1e-12,atol=1e-12)
        for k in range(len(self.result.phase)):
            self.assertAlmostEqual(float(np.sum(recovered[a['phase_index']==k])),self.result.summary['normal_force_n'])

    def test_sweep_contains_one_pair_snapshot_without_exceeding_source_peak(self):
        a=self.pattern.arrays
        # 瞬时窄带和全齿包络使用不同齿廓显示网格，不能逐像素直接比较。
        self.assertLessEqual(np.max(a['instant_pressure_pa']),self.pattern.summary['source_peak_pressure_pa'])
        self.assertLessEqual(np.max(a['sweep_pressure_pa']),self.pattern.summary['source_peak_pressure_pa'])
        self.assertLess(self.pattern.summary['snapshot_grid_force_relative_error'],.002)
        self.assertLess(self.pattern.summary['snapshot_selected_pair_force_n'],self.pattern.summary['snapshot_all_pair_force_n']+1e-12)

    def test_full_material_path_uses_multiple_wrapped_tooth_indices(self):
        a=self.pattern.arrays;p=self.result.pair
        self.assertGreater(len(np.unique(a['tooth_pair_index'])),1)
        low,high=p.contact_roll_bounds_m
        self.assertLess(np.min(a['roll_pinion_m'])-low,p.base_pitch_m/len(self.result.phase))
        self.assertLess(high-np.max(a['roll_pinion_m']),p.base_pitch_m/len(self.result.phase))
        self.assertGreater(np.max(a['roll_pinion_m'])-np.min(a['roll_pinion_m']),p.base_pitch_m)

    def test_internal_wheel_coordinate_is_not_mirrored_like_external_mesh(self):
        result=solve(internal=True)
        pattern=build_contact_pattern(result,member='wheel',profile_points=201)
        span=result.pair.center_distance_m*math.sin(result.pair.working_pressure_angle_rad)
        np.testing.assert_allclose(pattern.arrays['roll_wheel_m']-pattern.arrays['roll_pinion_m'],span)
        self.assertGreater(np.max(pattern.arrays['sweep_pressure_pa']),0)

    def test_zero_load_does_not_create_a_coloured_contact_patch(self):
        pattern=build_contact_pattern(solve(torque=0),profile_points=101)
        self.assertEqual(np.count_nonzero(pattern.arrays['sweep_pressure_pa']),0)
        self.assertEqual(pattern.summary['source_peak_pressure_pa'],0)

    def test_inactive_face_strips_are_not_filled_by_smoothing(self):
        pattern=build_contact_pattern(solve(torque=.01,crown=10e-6),profile_points=501)
        self.assertTrue(np.all(pattern.arrays['sweep_pressure_pa'][:,0]==0))
        self.assertGreater(np.max(pattern.arrays['sweep_pressure_pa']),0)

    def test_sparse_phase_holes_are_reported_and_reduce_with_actual_solves(self):
        coarse=build_contact_pattern(solve(phases=8),profile_points=501)
        fine=build_contact_pattern(solve(phases=128),profile_points=501)
        self.assertGreater(coarse.summary['unresolved_between_loaded_samples_fraction'],fine.summary['unresolved_between_loaded_samples_fraction'])
        self.assertGreater(np.count_nonzero(coarse.arrays['unresolved_sampling_mask']),0)

    def test_missing_phase_and_inconsistent_pressure_are_rejected(self):
        result=deepcopy(self.result);result.phase[1]=result.phase[0]
        with self.assertRaises(ValueError):build_contact_pattern(result)
        result=deepcopy(self.result);result.rotations[0].pressure_pa*=1.1
        with self.assertRaisesRegex(ValueError,'积分'):build_contact_pattern(result)

    def test_export_keeps_raw_arrays_without_pickle(self):
        with tempfile.TemporaryDirectory() as folder:
            export_contact_pattern(self.pattern,folder)
            with np.load(Path(folder)/'pattern.npz',allow_pickle=False) as a:
                np.testing.assert_array_equal(a['force_n'],self.pattern.arrays['force_n'])
                self.assertEqual(a['sweep_pressure_pa'].shape,(1001,7))

    def test_replot_rejects_modified_saved_pressure_data(self):
        from examples.build_contact_patterns import load_patterns
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);path=root/'data'/'case'/'sun_planet'/'planet_01'/'pinion'
            export_contact_pattern(self.pattern,path)
            (root/'summary.json').write_text(json.dumps({'patterns':[{'case':'case','mesh':'sun_planet','planet':1,'member':'pinion'}]}),encoding='utf-8')
            self.assertEqual(len(load_patterns(root)[0]),1)
            with (path/'pattern.npz').open('ab') as f:f.write(b'changed')
            with self.assertRaisesRegex(ValueError,'身份'):load_patterns(root)


if __name__=='__main__':unittest.main()
