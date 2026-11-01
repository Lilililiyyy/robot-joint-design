"""工程导出的参数、拓扑数量与SVG结构核验；不比较像素。"""
from copy import deepcopy
import csv
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from gear_contact import Gear
from robot_joint.analysis import JointResult
from robot_joint.engineering import export_engineering_parameters
from robot_joint.planetary import PlanetaryStage, stage_geometry


class EngineeringTests(unittest.TestCase):
    def setUp(self):
        stage = PlanetaryStage(18, 27, 72, 3, 0.0015, 0.012)
        self.inputs = {
            "case_id": 'NGW<&"示例',
            "stage": {"sun_teeth": 18, "planet_teeth": 27, "ring_teeth": 72,
                      "planet_count": 3, "module_m": 0.0015, "face_width_m": 0.012,
                      "pressure_angle_rad": math.radians(20)},
            "mesh": {"material": {"youngs_modulus_pa": 210e9, "poisson_ratio": 0.3}},
            "load_sharing": {"pin_stiffness_n_per_m": [8e8, 7e8, 6e8]},
            "bearings": {"spacing_m": 0.045, "load_offset_m": 0.012,
                         "ratings": [{"dynamic_capacity_n": 12000}, {"dynamic_capacity_n": 14000}]},
            "motor": {"continuous_torque_n_m": 8, "peak_torque_n_m": 16,
                      "maximum_speed_rad_s": 160},
            "drive": {"brake_torque_n_m": 12, "brake_safety_factor": 1.5},
            "precision": {"encoder_counts_per_revolution": 262144, "encoder_on_output": True},
        }
        self.result = JointResult(self.inputs, stage_geometry(stage), [], {}, {}, [], {}, {})
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.output = Path(self.temporary.name) / "带空格的工程输出"

    def export(self):
        paths = export_engineering_parameters(self.result, self.output)
        self.assertEqual({Path(p).name for p in paths}, {"cad_parameters.json", "bom.csv", "layout.svg"})
        self.assertTrue(all(Path(p).is_file() for p in paths))
        return json.loads((self.output / "cad_parameters.json").read_text(encoding="utf-8"))

    def test_parameters_reproduce_default_gear_radii_and_placement(self):
        parameters = self.export()
        self.assertEqual(parameters["stage"], self.inputs["stage"])
        self.assertEqual(parameters["units"]["length"], "m")
        self.assertFalse(parameters["solid_constructed"])
        self.assertFalse(parameters["manufacturing_tooth_profile_constructed"])
        for name, teeth, internal in (("sun", 18, False), ("planet", 27, False), ("ring", 72, True)):
            gear = Gear(teeth, 0.0015, 0.012, pressure_angle_rad=math.radians(20), internal=internal)
            radii = parameters["gear_radii_m"][name]
            for field in ("reference_radius_m", "base_radius_m", "tip_radius_m", "root_radius_m"):
                self.assertAlmostEqual(radii[field], getattr(gear, field), places=15)
        centres = np.array(parameters["geometry"]["planet_centres_xy_m"])
        np.testing.assert_allclose(np.linalg.norm(centres, axis=1), (18+27)*0.0015/2, atol=1e-15)
        self.assertAlmostEqual(parameters["bearings"]["centre_z_m"][1]
                               - parameters["bearings"]["centre_z_m"][0], self.inputs["bearings"]["spacing_m"])

    def test_bom_counts_and_specs_are_input_derived(self):
        self.export()
        with (self.output / "bom.csv").open(encoding="utf-8-sig", newline="") as stream:
            rows = {row["item_id"]: row for row in csv.DictReader(stream)}
        expected = {"sun": 1, "planet": 3, "ring": 1, "carrier": 1, "planet_pin": 3,
                    "output_bearing": 2, "motor": 1, "brake": 1, "encoder": 1}
        self.assertEqual({key: int(row["quantity"]) for key, row in rows.items()}, expected)
        self.assertTrue(all(row["procurement_model"] == "" for row in rows.values()))
        self.assertEqual(json.loads(rows["motor"]["specification"]), self.inputs["motor"])
        self.assertEqual(json.loads(rows["output_bearing"]["specification"]), self.inputs["bearings"])
        self.assertEqual(json.loads(rows["planet_pin"]["specification"])["tangential_stiffness_n_per_m"],
                         self.inputs["load_sharing"]["pin_stiffness_n_per_m"])
        self.assertEqual(json.loads(rows["ring"]["specification"])["teeth"], 72)
        self.assertTrue(json.loads(rows["ring"]["specification"])["internal"])

    def test_svg_contains_exact_layout_metadata_without_external_resources(self):
        parameters = self.export()
        body = (self.output / "layout.svg").read_text(encoding="utf-8")
        tree = ET.fromstring(body)
        namespace = {"s": "http://www.w3.org/2000/svg"}
        self.assertIn(self.inputs["case_id"], tree.find("s:title", namespace).text)
        circles = {element.attrib["id"]: element for element in tree.findall(".//s:circle", namespace)}
        self.assertEqual(len([key for key in circles if key.startswith("planet_") and key.endswith("_reference")]), 3)
        for index, centre in enumerate(parameters["geometry"]["planet_centres_xy_m"], 1):
            circle = circles[f"planet_{index}_reference"]
            self.assertAlmostEqual(float(circle.attrib["data-centre-x-m"]), centre[0])
            self.assertAlmostEqual(float(circle.attrib["data-centre-y-m"]), centre[1])
        self.assertAlmostEqual(float(circles["ring_tip"].attrib["data-radius-m"]), 0.0015*(72-2)/2)
        self.assertAlmostEqual(float(circles["ring_root"].attrib["data-radius-m"]), 0.0015*(72+2.5)/2)
        self.assertIn("不是制造齿廓", tree.find("s:desc", namespace).text)
        self.assertNotIn("<script", body)
        self.assertNotIn("href=", body)
        self.assertNotIn("@font-face", body)

    def test_export_does_not_mutate_inputs_and_updates_geometry_from_another_valid_case(self):
        original = deepcopy(self.inputs)
        self.export()
        self.assertEqual(self.inputs, original)
        self.inputs["stage"].update(sun_teeth=20, planet_teeth=30, ring_teeth=80, planet_count=4)
        self.result.geometry = stage_geometry(PlanetaryStage(**self.inputs["stage"]))
        self.inputs["load_sharing"]["pin_stiffness_n_per_m"] = 9e8
        parameters = self.export()
        self.assertEqual(len(parameters["geometry"]["planet_centres_xy_m"]), 4)
        with (self.output / "bom.csv").open(encoding="utf-8-sig", newline="") as stream:
            rows = {row["item_id"]: row for row in csv.DictReader(stream)}
        self.assertEqual(int(rows["planet"]["quantity"]), 4)
        self.assertEqual(int(rows["planet_pin"]["quantity"]), 4)


if __name__ == "__main__":
    unittest.main()
