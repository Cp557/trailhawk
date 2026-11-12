import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from balance import DESIGN_AIR, hover_shares
from performance import ELECTRONICS_WATTS, ORBIT_RADIUS, G, cruise, hover, mission, weight
from sim import ORBIT_HEIGHT, aircraft
from vtol import build_vtol

NODE = shutil.which("node")


@unittest.skipUnless(NODE, "needs Node.js to fly sim/check.mjs")
class SimTest(unittest.TestCase):
    """The simulator flies the same aircraft as the reports (sim/check.mjs flies it headless)."""

    @classmethod
    def setUpClass(cls):
        cls.vtol = build_vtol()
        with tempfile.TemporaryDirectory() as folder:
            data = Path(folder) / "aircraft.json"
            data.write_text(json.dumps(aircraft(cls.vtol)))
            check = Path(__file__).parent / "sim" / "check.mjs"
            cls.results = json.loads(subprocess.run([NODE, str(check), str(data)], capture_output=True, text=True, check=True).stdout)

    def test_hover_thrusts_match_the_balance(self):
        for (name, share), grams in zip(hover_shares(self.vtol).items(), self.results["hover"]["thrust_grams"]):
            with self.subTest(motor=name):
                self.assertAlmostEqual(grams / (share * weight(self.vtol) / G * 1000), 1, delta=0.02)
        self.assertAlmostEqual(self.results["hover"]["altitude_change"], 0, delta=0.5)
        hover_watts = sum(watts for _, _, watts in hover(self.vtol, DESIGN_AIR).values()) + ELECTRONICS_WATTS
        self.assertAlmostEqual(self.results["hover"]["watts"] / hover_watts, 1, delta=0.03)

    def test_cruise_power_matches_the_performance_model(self):
        level = self.results["cruise"]
        self.assertAlmostEqual(level["airspeed"], 15.5, delta=0.2)
        self.assertAlmostEqual(level["climb"], 0, delta=0.2)
        self.assertAlmostEqual(level["watts"] / cruise(self.vtol, 15.5, DESIGN_AIR)[0], 1, delta=0.1)

    def test_circles_the_point_at_the_mission_radius_height_and_power(self):
        orbit = self.results["orbit"]
        self.assertAlmostEqual(orbit["radius"] / ORBIT_RADIUS, 1, delta=0.05)
        self.assertLess(orbit["farthest_off"], 5)
        self.assertGreater(orbit["laps_per_minute"], 1)  # clockwise
        self.assertAlmostEqual(orbit["height"], ORBIT_HEIGHT, delta=1)  # climbed from 40 m
        self.assertAlmostEqual(orbit["altitude_change"], 0, delta=1)
        self.assertAlmostEqual(orbit["watts"] / mission(self.vtol, DESIGN_AIR)["loiter_watts"], 1, delta=0.1)

    def test_dive_key_while_circling_lowers_the_held_height(self):
        orbit = self.results["orbit"]
        self.assertTrue(orbit["circling_after_dive"])
        self.assertLess(orbit["height_after_dive"], ORBIT_HEIGHT - 8)  # 5 s at 3 m/s, not climbing back
        self.assertAlmostEqual(orbit["climb_after_dive"], 0, delta=0.3)

    def test_keys_do_what_the_help_says(self):
        # Hover: up moves forward (north), right moves right (east), D yaws right, W climbs.
        # Plane: up dives, right banks into a right turn, D yaws right, W speeds up.
        keys = self.results["keys"]
        for key, least in (("hover_up", 1), ("hover_right", 1), ("hover_d", 0.2), ("hover_w", 2), ("plane_right", 0.3), ("plane_d", 0.05), ("plane_w", 1)):
            with self.subTest(key=key):
                self.assertGreater(keys[key], least)
        self.assertLess(keys["plane_up"], -2)

    def test_takes_off_transitions_both_ways_and_lands(self):
        trip = self.results["trip"]
        self.assertIsNone(trip["crashed"])
        self.assertGreater(trip["takeoff_altitude"], 35)
        self.assertIsNotNone(trip["seconds_to_plane"])
        self.assertLess(trip["seconds_to_plane"], 15)
        self.assertGreater(trip["lowest"], trip["takeoff_altitude"] - 10)
        self.assertLess(trip["highest"], trip["takeoff_altitude"] + 15)
        self.assertEqual(trip["final_mode"], "hover")
        self.assertLess(trip["final_speed"], 1)
        self.assertEqual(trip["landed"], "landed")


if __name__ == "__main__":
    unittest.main()
