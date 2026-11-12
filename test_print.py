import unittest

from build123d import Box, Pos
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps

from print_parts import fits_bed, print_pieces, printed_parts
from vtol import build_vtol, components, spar_lines, tube


def volume(shape):
    """Volume by adaptive integration: build123d's default undercounts the lofted airfoils."""
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, properties, 1e-4)
    return properties.Mass()


class PrintTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        vtol = build_vtol()
        cls.prepared = printed_parts(vtol)
        cls.pieces = print_pieces(vtol)

    def test_pieces_are_valid_single_solids_that_fit_the_bed(self):
        for name, piece in self.pieces.items():
            with self.subTest(piece=name):
                self.assertTrue(piece.is_valid)
                self.assertEqual(len(piece.solids()), 1)
                self.assertTrue(fits_bed(piece))

    def test_cutting_keeps_all_the_material(self):
        for label, (solid, *_) in self.prepared.items():
            pieces = [piece for name, piece in self.pieces.items() if name.rsplit("_", 1)[0] == label]
            with self.subTest(part=label):
                self.assertAlmostEqual(sum(map(volume, pieces)), volume(solid), delta=volume(solid) * 1e-3)

    def test_spars_and_components_sit_in_channels_and_pockets(self):
        hardware = [tube(start, end, r) for start, end, r in spar_lines().values()]
        hardware += [Pos(center) * Box(*size) for _, center, size in components().values()]
        for name, piece in self.pieces.items():
            box = piece.bounding_box()
            for part in hardware:
                if box.intersects(part.bounding_box()):
                    common = piece & part
                    with self.subTest(piece=name):
                        self.assertAlmostEqual(common.volume if common else 0, 0, delta=1e-6)

    def test_sleeves_line_every_hollow_joint(self):
        sleeves = {name: piece for name, piece in self.pieces.items() if "_sleeve_" in name}
        self.assertTrue(sleeves)
        shells = [piece for name, piece in self.pieces.items() if name.startswith(("fuselage_", "nacelles_")) and name not in sleeves]
        for name, sleeve in sleeves.items():
            for shell in shells:
                common = sleeve & shell
                with self.subTest(sleeve=name):
                    self.assertAlmostEqual(common.volume if common else 0, 0, delta=1)

    def test_battery_and_computer_lift_out_through_the_hatch(self):
        for label in ("battery", "computer"):
            _, center, (length, width, height) = components()[label]
            path = Pos(center.X, center.Y, center.Z + 150) * Box(length, width, 300 + height)
            for name, piece in self.pieces.items():
                if name.startswith("fuselage_"):
                    common = piece & path
                    with self.subTest(component=label, piece=name):
                        self.assertAlmostEqual(common.volume if common else 0, 0, delta=1e-6)


if __name__ == "__main__":
    unittest.main()
