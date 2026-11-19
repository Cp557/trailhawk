import unittest

from drone.balance import (
    CG_RANGE, FIN_VOLUME_RANGE, TAIL_VOLUME_RANGE, center_of_gravity, cg_fraction, hover_shares, inertia, mass_items,
    mean_aerodynamic_chord, tail_volumes,
)  # fmt: skip
from drone.vtol import build_vtol, chord_point, wing_stations


class BalanceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vtol = build_vtol()
        cls.items = mass_items(cls.vtol)

    def test_every_part_has_a_positive_mass(self):
        for label, grams, _ in self.items:
            with self.subTest(part=label):
                self.assertGreater(grams, 0)

    def test_inertia_is_physical(self):
        tensor = inertia(self.vtol)
        self.assertTrue((abs(tensor - tensor.T) < 1e-12).all())
        roll, pitch, yaw = tensor.diagonal()  # CAD X, Y, Z: roll, pitch and yaw axes
        self.assertTrue(roll < pitch + yaw and pitch < roll + yaw and yaw < roll + pitch)
        self.assertGreater(yaw, max(roll, pitch))  # a flat aircraft: mass spreads in both span and length

    def test_mean_aerodynamic_chord_lies_within_the_wing(self):
        mac, mac_le, area = mean_aerodynamic_chord()
        root, crank, tip = wing_stations()
        self.assertTrue(tip[1] < mac < root[1])
        self.assertTrue(chord_point(root, 0).X < mac_le < chord_point(tip, 0).X)
        panel = lambda a, b: (a[1] + b[1]) / 2 * (b[0].origin.Y - a[0].origin.Y)  # noqa: E731
        self.assertAlmostEqual(area, 2 * (panel(root, crank) + panel(crank, tip)))

    def test_cruise_cg_is_in_the_usual_range(self):
        fraction = cg_fraction(center_of_gravity(self.items))
        self.assertTrue(CG_RANGE[0] <= fraction <= CG_RANGE[1], f"{fraction:.1%} of MAC")

    def test_cg_is_on_the_centerline(self):
        self.assertAlmostEqual(center_of_gravity(self.items).Y, 0, delta=1)

    def test_hover_load_is_shared_without_overloading_a_motor(self):
        shares = hover_shares(self.vtol)
        self.assertAlmostEqual(sum(shares.values()), 1)
        for name, share in shares.items():
            with self.subTest(motor=name):
                self.assertTrue(0.25 <= share <= 0.40, f"{share:.1%}")

    def test_tails_are_big_enough_for_the_longer_wing(self):
        horizontal, vertical = tail_volumes(center_of_gravity(self.items))
        self.assertTrue(TAIL_VOLUME_RANGE[0] <= horizontal <= TAIL_VOLUME_RANGE[1], horizontal)
        self.assertTrue(FIN_VOLUME_RANGE[0] <= vertical <= FIN_VOLUME_RANGE[1], vertical)


if __name__ == "__main__":
    unittest.main()
