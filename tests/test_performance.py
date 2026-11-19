import unittest

from trailhawk.balance import DESIGN_AIR, SEA_LEVEL, Air, stall_speed
from trailhawk.parts import BATTERY, ESC, MOTOR, TILT_SERVO
from trailhawk.performance import (
    G, HOVER_SECONDS, LOITER_MINUTES, SAG_CELL_VOLTS, SERVO_MARGIN, SPEED_MARGIN, THRUST_MARGIN, heaviest_takeoff, hottest_day, hover,
    hover_need, max_static, mission, motor_test, parasite_drag_area, prop_power, prop_thrust, speeds, tilt_servo_load, tilt_servo_torque,
    thinnest_air, weight, wing_reference,
)  # fmt: skip
from trailhawk.vtol import YAW_TILT, build_vtol


class PerformanceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vtol = build_vtol()
        cls.results = mission(cls.vtol, DESIGN_AIR)

    def test_prop_data_matches_the_motor_test_where_we_fly(self):
        # APC's coefficients come from its 6000 rpm block; hover and cruise run near it. The test is at sea level.
        for throttle, _, _, _, rpm, shaft, thrust in zip(*motor_test()):
            if throttle <= 60:
                with self.subTest(throttle=throttle):
                    self.assertAlmostEqual(prop_thrust(rpm, 0, SEA_LEVEL) / thrust, 1, delta=0.07)
                    self.assertAlmostEqual(prop_power(rpm, 0, SEA_LEVEL) / shaft, 1, delta=0.07)

    def test_hover_thrust_carries_the_weight(self):
        self.assertAlmostEqual(sum(thrust for thrust, _, _ in hover(self.vtol, DESIGN_AIR).values()), weight(self.vtol))

    def test_full_throttle_matches_the_motor_test_at_sea_level(self):
        tested_volts, *_ = motor_test()[1][-1:]
        thrust, amps = max_static(tested_volts, SEA_LEVEL)
        self.assertAlmostEqual(thrust, motor_test()[6][-1])
        self.assertAlmostEqual(amps, motor_test()[2][-1])

    def test_thin_air_costs_thrust(self):
        volts = BATTERY["cells"] * SAG_CELL_VOLTS
        self.assertLess(max_static(volts, DESIGN_AIR)[0], max_static(volts, SEA_LEVEL)[0])

    def test_motors_have_thrust_margin_on_a_sagging_battery_at_altitude(self):
        thrust, _ = max_static(BATTERY["cells"] * SAG_CELL_VOLTS, DESIGN_AIR)
        for name, (hover_thrust, _, _) in hover(self.vtol, DESIGN_AIR).items():
            with self.subTest(motor=name):
                self.assertGreaterEqual(thrust, THRUST_MARGIN * hover_thrust)

    def test_hover_limit_is_where_full_throttle_just_meets_the_margin(self):
        limit = thinnest_air(self.vtol)
        thrust, _ = max_static(BATTERY["cells"] * SAG_CELL_VOLTS, Air(limit, SEA_LEVEL.viscosity))
        self.assertAlmostEqual(thrust / hover_need(self.vtol), 1, delta=1e-6)
        self.assertLessEqual(limit, DESIGN_AIR.density)
        self.assertGreaterEqual(heaviest_takeoff(self.vtol, DESIGN_AIR), weight(self.vtol) / G * 1000)

    def test_hottest_day_matches_the_design_air(self):
        # DESIGN_AIR is 12,000 ft on a 10 C day
        self.assertAlmostEqual(hottest_day(DESIGN_AIR.density, 12000), 10, delta=0.2)

    def test_full_throttle_stays_within_the_motor_and_esc(self):
        _, amps = max_static(BATTERY["cells"] * BATTERY["cell_volts"], SEA_LEVEL)
        self.assertLessEqual(amps, min(MOTOR["peak_amps"], ESC["amps"]))

    def test_hover_current_stays_within_the_battery(self):
        amps = self.results["hover_watts"] / (BATTERY["cells"] * SAG_CELL_VOLTS)
        self.assertLessEqual(amps, BATTERY["max_amps"] / 2)

    def test_esc_suits_the_battery(self):
        low, high = ESC["cells"]
        self.assertTrue(low <= BATTERY["cells"] <= high)

    def test_tilt_servo_holds_and_reaches(self):
        self.assertGreaterEqual(tilt_servo_torque(), SERVO_MARGIN * tilt_servo_load(self.vtol))
        self.assertGreaterEqual(TILT_SERVO["travel"], 90 + YAW_TILT)

    def test_drag_is_plausible_for_a_small_uav(self):
        area, _ = wing_reference()
        cd0 = parasite_drag_area(self.vtol, self.results["range_speed"], DESIGN_AIR)[0] / area
        self.assertTrue(0.02 < cd0 < 0.1, cd0)

    def test_cruise_takes_a_fraction_of_hover_power(self):
        self.assertLess(self.results["range_watts"], 0.4 * self.results["hover_watts"])

    def test_cruise_speeds_respect_the_stall_margin(self):
        stall = stall_speed(weight(self.vtol) / G * 1000, DESIGN_AIR)
        self.assertGreaterEqual(speeds(self.vtol, DESIGN_AIR)[0], SPEED_MARGIN * stall)
        self.assertLessEqual(self.results["endurance_speed"], self.results["range_speed"])

    def test_circles_long_enough_at_altitude(self):
        self.assertGreaterEqual(self.results["loiter_minutes"], LOITER_MINUTES)

    def test_circling_costs_more_than_flying_straight(self):
        self.assertGreater(self.results["loiter_watts"], self.results["endurance_watts"])

    def test_missions_spend_exactly_the_usable_energy(self):
        for watts, minutes in (("range_watts", "cruise_minutes"), ("loiter_watts", "loiter_minutes")):
            with self.subTest(watts):
                spent = self.results["hover_watts"] * HOVER_SECONDS + self.results[watts] * self.results[minutes] * 60
                self.assertAlmostEqual(spent, self.results["energy"])
                self.assertGreater(self.results[minutes], 0)


if __name__ == "__main__":
    unittest.main()
