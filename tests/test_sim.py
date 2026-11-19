import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from drone.balance import DESIGN_AIR, hover_shares
from drone.performance import ELECTRONICS_WATTS, ORBIT_RADIUS, G, cruise, hover, mission, weight
from drone.sim import ORBIT_HEIGHT, aircraft
from drone.vtol import build_vtol

NODE = shutil.which("node")
SITE_DATA = Path(__file__).parents[1] / "site" / "data"


def rounded(value):
    """JSON data with every float to 9 significant figures: the CAD kernel's volume integrals vary in
    their last bits from run to run."""
    if isinstance(value, dict):
        return {key: rounded(v) for key, v in value.items()}
    if isinstance(value, list):
        return [rounded(v) for v in value]
    return float(f"{value:.9g}") if isinstance(value, float) else value


@unittest.skipUnless(NODE, "needs Node.js to fly tests/check.mjs")
class SimTest(unittest.TestCase):
    """The simulator flies the same aircraft as the reports (check.mjs flies it headless)."""

    @classmethod
    def setUpClass(cls):
        cls.vtol = build_vtol()
        cls.aircraft = json.loads(json.dumps(aircraft(cls.vtol)))
        with tempfile.TemporaryDirectory() as folder:
            data = Path(folder) / "aircraft.json"
            data.write_text(json.dumps(cls.aircraft))
            check = Path(__file__).parent / "check.mjs"
            cls.results = json.loads(subprocess.run([NODE, str(check), str(data)], capture_output=True, text=True, check=True).stdout)

    def test_site_data_matches_the_model(self):
        saved = json.loads((SITE_DATA / "aircraft.json").read_text())
        self.assertEqual(rounded(saved), rounded(self.aircraft), "site/data/ is out of date: python -m drone.sim --export")

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

    def test_sticks_move_it_the_right_way(self):
        # Hover: pitch forward moves forward (north), roll right moves right (east), yaw right turns
        # right, throttle up climbs. Plane: pitch forward dives, roll right banks into a right turn,
        # yaw right turns right, throttle up speeds up.
        sticks = self.results["sticks"]
        for stick, least in (("hover_forward", 1), ("hover_right", 1), ("hover_yaw", 0.2), ("hover_climb", 2), ("plane_bank", 0.3), ("plane_rudder", 0.05), ("plane_faster", 1)):
            with self.subTest(stick=stick):
                self.assertGreater(sticks[stick], least)
        self.assertLess(sticks["plane_dive"], -2)

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

    def test_sits_lands_and_circles_on_made_up_terrain(self):
        slopes = self.results["slopes"]
        self.assertLess(slopes["crept"], 0.01)  # m in a minute, landed on a 5 deg slope
        self.assertEqual((slopes["slope_landing"], slopes["slope_crashed"]), ("landed", None))
        self.assertEqual((slopes["valley_landing"], slopes["valley_crashed"]), ("landed", None))  # 30 m below the pad
        self.assertAlmostEqual(slopes["raised_orbit_height"], 20 + ORBIT_HEIGHT, delta=1)  # over ground 20 m up
        self.assertEqual(slopes["ridge"], "hit the ground")

    def test_generated_worlds_are_repeatable_walkable_and_wet(self):
        self.assertTrue(self.results["repeatable"])  # the same seed makes the same world
        for world in self.results["worlds"]:
            with self.subTest(seed=world["seed"]):
                self.assertLess(world["pad"], 0.01)  # m: the pad's clearing is flat
                self.assertGreater(world["trail_length"], 500)
                self.assertLess(world["steepest_trail"], 0.17)  # over 20 m; graded to 0.15 per 2 m
                self.assertGreater(world["trail_above_water"], 1)  # it ends at the shore
                self.assertGreater(world["trees"], 1000)
                self.assertGreaterEqual(world["tree_to_trail"], 5)
                self.assertGreaterEqual(world["tree_to_pad"], 35)
                self.assertGreaterEqual(world["tree_above_water"], 1.5)
                self.assertGreater(world["lake_area"], 10000)  # m²
                self.assertGreater(world["rim_above_water"], 5)  # m: the hills hide the water disc's edge

    def test_follows_the_walking_hiker_to_the_lake(self):
        # The page's own flight: circling him from the start, following him until after he reaches the lake
        for world in self.results["worlds"]:
            flight = world["flight"]
            if flight is None:
                continue
            with self.subTest(seed=world["seed"]):
                self.assertIsNone(flight["crashed"])
                self.assertEqual(flight["mode"], "plane")
                self.assertTrue(flight["circling"])
                self.assertAlmostEqual(flight["radius"] / ORBIT_RADIUS, 1, delta=0.03)  # from where he really is
                self.assertLess(flight["farthest_off"], 20)  # m: his phone's error, and him walking on
                self.assertAlmostEqual(flight["above_hiker"], ORBIT_HEIGHT, delta=1.5)
                self.assertGreater(flight["lowest"], 50)  # m above the ground below
                self.assertLess(flight["highest"], 120)  # the 400 ft ceiling, all the way to the lake

    def test_cameras_see_what_the_model_says(self):
        # 60 looks at an elk 75 m out from 100 m up, as when circling
        sensors = self.results["sensors"]
        self.assertEqual(sensors["even_odds"], 0.5)  # ACQUIRE's curve, at Johnson's 50% point
        self.assertAlmostEqual(sensors["thermal_pixels"], 2.46, delta=0.01)  # across the elk, 124 m away
        self.assertGreater(sensors["open"]["thermal"], 25)  # about two looks in three
        self.assertGreater(sensors["open"]["color"], 55)  # named nearly every look
        self.assertLess(sensors["forest"]["thermal"], sensors["open"]["thermal"] / 2)  # full forest hides most of it
        self.assertLess(sensors["forest"]["color"], 15)
        self.assertEqual(sensors["ridge"], {"thermal": 0, "color": 0})  # the ground in the way
        self.assertEqual(sensors["behind"], {"thermal": 0, "color": 0})  # out of view

    def test_maps_the_hike(self):
        # What the cameras found on the follow flights to the lake, in this model
        for world in self.results["worlds"]:
            if world["flight"] is None:
                continue
            found = world["flight"]["cameras"]
            with self.subTest(seed=world["seed"]):
                self.assertEqual(found["pins_never_in_view"], 0)  # every pin came from a camera's view
                self.assertEqual(found["bears"]["named"], found["bears"]["of"])  # by the trail ahead
                self.assertEqual(found["bears_warned_of"], found["bears"]["named"])  # each gets the page's warning
                self.assertEqual(found["rocks"]["ruled_out"], found["rocks"]["flagged"])  # each false alarm cleared
                self.assertTrue(found["lake_mapped"])
                self.assertGreater(found["seen_near_trail"], 0.8)  # of the ground within 150 m of the trail
                self.assertGreaterEqual(found["within_100"]["named"] + found["from_100_to_200"]["named"], 3)


if __name__ == "__main__":
    unittest.main()
