import unittest

from pack import PACK, field_sections, world_parts
from print_parts import fits_bed
from vtol import build_vtol


class PackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vtol = build_vtol()
        cls.sections = field_sections(cls.vtol)

    def test_every_section_fits_the_pack(self):
        for name, section in self.sections.items():
            with self.subTest(section=name):
                self.assertTrue(fits_bed(section, PACK))

    def test_the_big_parts_come_apart(self):
        for name in ("pod", "boom", "tail", "rear_motor", "wing_inner_right", "wing_inner_left", "wing_outer_right", "wing_outer_left", "skid_right", "skid_left"):
            with self.subTest(section=name):
                self.assertIn(name, self.sections)

    def test_no_part_is_left_out(self):
        whole = sum(len(shape.solids()) for _, shape in world_parts(self.vtol))
        packed = sum(len(section.solids()) for section in self.sections.values())
        # The wing, fuselage, nacelles and skids are each cut into more than one piece
        self.assertGreater(packed, whole)


if __name__ == "__main__":
    unittest.main()
