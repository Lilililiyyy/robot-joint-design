"""实物需求参照案例的动力学、来源与展示数据一致性检查。"""
from copy import deepcopy
import json
import math
from pathlib import Path
import unittest
import numpy as np
from examples.pendulum_case import pendulum_duty,candidate_configs,validate_saved_duty
from examples.build_showcase import dense_motion,FIGURES


def root():
    return next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file())


class PendulumCaseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base=json.loads((root()/'examples/dfki_pendulum_case.json').read_text(encoding='utf-8'))
        cls.req=json.loads((root()/'examples/dfki_pendulum_requirements.json').read_text(encoding='utf-8'))

    def test_generated_states_match_saved_solver_input_with_declared_tolerance(self):
        segments,derived=pendulum_duty(self.req)
        validate_saved_duty(segments,self.base['duty_segments'])
        self.assertEqual(len(segments),34)
        self.assertAlmostEqual(derived['total_mass_kg'],.676)
        self.assertAlmostEqual(derived['inertia_kg_m2'],(.546+.13/3)*.5**2)
        self.assertAlmostEqual(derived['period_s'],1.8)

    def test_last_bit_math_library_difference_is_accepted(self):
        saved=deepcopy(self.base['duty_segments'])
        saved[3]['output_torque_n_m']=float(np.nextafter(saved[3]['output_torque_n_m'],math.inf))
        saved[3]['output_wrench'][5]=saved[3]['output_torque_n_m']
        validate_saved_duty(pendulum_duty(self.req)[0],saved)

    def test_actual_input_label_or_layout_changes_are_rejected(self):
        for key in ('torque','label','layout'):
            saved=deepcopy(self.base['duty_segments'])
            if key=='torque':saved[3]['output_torque_n_m']+=1e-6
            elif key=='label':saved[3]['label']='changed'
            else:saved[3]['output_wrench']=saved[3]['output_wrench'][:5]
            with self.subTest(key=key),self.assertRaises(ValueError):
                validate_saved_duty(pendulum_duty(self.req)[0],saved)

    def test_dense_trajectory_is_periodic_and_stops_during_holds(self):
        t,q,w,a,torque=dense_motion(self.req)
        self.assertAlmostEqual(q[0],q[-1]);self.assertEqual(w[0],0);self.assertEqual(w[-1],0)
        held=(t>.5)&(t<.9)
        np.testing.assert_array_equal(w[held],0)
        np.testing.assert_allclose(torque[held],.676*9.81*(.45192307692307687)*math.sin(math.pi/3))

    def test_inverse_dynamics_and_load_signs_are_consistent(self):
        segments,derived=pendulum_duty(self.req)
        for s,state in zip(segments,derived['states']):
            expected=derived['inertia_kg_m2']*state['acceleration_rad_s2']+derived['gravity_first_moment_n_m']*math.sin(state['theta_rad'])+.02*state['speed_rad_s']
            self.assertAlmostEqual(s['output_torque_n_m'],expected)
            self.assertEqual(s['output_wrench'][5],s['output_torque_n_m'])
        powers=[s['output_torque_n_m']*s['output_speed_rad_s'] for s in segments]
        self.assertTrue(any(p>0 for p in powers));self.assertTrue(any(p<0 for p in powers))
        dense_peak=np.max(np.abs(dense_motion(self.req)[-1]))
        self.assertLess(abs(max(abs(s['output_torque_n_m']) for s in segments)-dense_peak)/dense_peak,.003)

    def test_sources_and_model_assumptions_remain_separate(self):
        self.assertIn('11a946d4e6db58d001fdeff89be269eb61461757',self.req['sources'][0]['url'])
        self.assertEqual(self.req['source_parameters']['gear_sun_planet_ring'],[20,40,100])
        a=.0005*(20+40)/2
        self.assertAlmostEqual(a,self.req['source_conflicts']['adopted_sun_planet_center_distance_m'])
        self.assertNotEqual(a,self.req['source_conflicts']['mjbots_published_center_distance_m'])

    def test_candidates_preserve_requirements_and_geometry(self):
        before=deepcopy(self.base);candidates=candidate_configs(self.base)
        self.assertEqual(self.base,before)
        for c in candidates:
            self.assertEqual(c['stage'],self.base['stage']);self.assertEqual(c['duty_segments'],self.base['duty_segments'])
        self.assertNotEqual(candidates[1]['load_sharing'],candidates[0]['load_sharing'])

    def test_sixth_ratio_and_consistent_copper_loss_basis(self):
        s=self.base['stage'];self.assertEqual(1+s['ring_teeth']/s['sun_teeth'],6)
        self.assertEqual((s['sun_teeth']+s['ring_teeth'])%s['planet_count'],0)
        self.assertAlmostEqual(self.base['motor']['resistance_ohm'],(.091/.2206)**2)
        self.assertEqual(len(FIGURES),9)


if __name__=='__main__':unittest.main()
