import unittest

import numpy as np

from performance import weight
from strength import checks, landing_load_factor, lift_per_mm, limit_load_factor, wing_moment
from vtol import SPAR_END_Y, WING_JOINTS, WING_TIP_Y, build_vtol


class StrengthTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vtol = build_vtol()

    def test_every_check_has_a_positive_margin(self):
        for name, demand, allowable, unit in checks(self.vtol):
            with self.subTest(check=name):
                self.assertGreater(demand, 0, unit)
                self.assertLess(demand, allowable, f"{demand:.1f} of {allowable:.0f} {unit}")

    def test_lift_distribution_carries_the_load(self):
        span = np.linspace(0, WING_TIP_Y, 1000)
        half = np.trapezoid(lift_per_mm(span, 100), span)
        self.assertAlmostEqual(half, 50, delta=0.5)

    def test_moment_falls_toward_the_tip(self):
        stations = [0, *WING_JOINTS, SPAR_END_Y]
        moments = [wing_moment(y, weight(self.vtol)) for y in stations]
        self.assertEqual(moments, sorted(moments, reverse=True))

    def test_design_loads_are_realistic(self):
        self.assertTrue(3 <= limit_load_factor(self.vtol) <= 8)
        self.assertTrue(3 <= landing_load_factor() <= 10)


if __name__ == "__main__":
    unittest.main()
