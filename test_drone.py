import unittest

from drone import PROP_RADIUS, WING_HALF_SPAN, build_drone


class DroneTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.drone = build_drone()
        cls.parts = {part.label: part for part in cls.drone.children}

    def test_every_part_is_one_valid_solid(self):
        self.assertEqual(len(self.parts), 7)
        for label, part in self.parts.items():
            with self.subTest(part=label):
                self.assertTrue(part.is_valid)
                self.assertEqual(len(part.solids()), 1)
                self.assertGreater(part.volume, 0)

    def test_wing_span_and_symmetry(self):
        box = self.parts["wing"].bounding_box()
        self.assertAlmostEqual(box.min.Y, -WING_HALF_SPAN, delta=1)
        self.assertAlmostEqual(box.max.Y, WING_HALF_SPAN, delta=1)

    def test_propeller_diameter(self):
        box = self.parts["propeller"].bounding_box()
        self.assertAlmostEqual(box.size.Z, 2 * PROP_RADIUS, delta=2)


if __name__ == "__main__":
    unittest.main()
