import unittest
from math import pi

from build123d import Box, Cylinder, Location, Pos, Rot

import fly
from vtol import (
    GROUND_Z, HINGE_UP, HUB_X, MOTORS, PROP_GAP, PROP_RADIUS, SKIN, TILTING, YAW_TILT, build_vtol, components, hinge,
    spar_lines, tube,
)  # fmt: skip

AIRFRAME = ("fuselage", "wing", "horizontal_tail", "fin", "nacelles", "tail_nacelle", "landing_gear")


def leaves(node, prefix=""):
    """(path, shape) for every leaf of an assembly."""
    for child in node.children:
        path = f"{prefix}/{child.label}"
        if child.children:
            yield from leaves(child, path)
        else:
            yield path, child


def overlap(a, b):
    common = a & b
    return sum(solid.volume for solid in common.solids()) if common else 0


def tilts(name):
    """Tilt angles a motor goes through: the front motors from cruise to past vertical for yaw,
    the rear motor fixed upright."""
    return (0, 30, 60, 90, 90 + YAW_TILT) if name in TILTING else (90,)


class VtolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vtol = build_vtol()
        cls.parts = {part.label: part for part in cls.vtol.children}

    def test_leaves_are_valid(self):
        expected = {
            "/fuselage", "/wing", "/horizontal_tail", "/fin",
            "/aileron_right", "/aileron_left", "/elevator_right", "/elevator_left", "/rudder",
            "/nacelles", "/tail_nacelle", "/landing_gear", "/center_fitting", "/details", "/gimbal_mount", "/gimbal/camera", "/gimbal/lens", "/gimbal/thermal_lens",
            *(f"/{motor}/{part}" for motor in MOTORS for part in ("mount", "motor", "prop_nut", "propeller")),
            *(f"/hinges/{motor}_hinge/{part}" for motor in MOTORS for part in ("bearing", "hinge_pin")),
            *(f"/hinges/{motor}_hinge/tilt_servo" for motor in TILTING),
            *(f"/spars/{spar}" for spar in spar_lines()),
            *(f"/avionics/{component}" for component in components()),
        }  # fmt: skip
        found = dict(leaves(self.vtol))
        self.assertEqual(set(found), expected)
        for path, shape in found.items():
            with self.subTest(path=path):
                self.assertTrue(shape.is_valid)
                self.assertGreater(shape.volume, 0)

    def test_motors_sit_on_their_hinges(self):
        for name, (nose, _) in MOTORS.items():
            with self.subTest(motor=name):
                self.assertEqual(tuple(self.parts[name].location.position), hinge(nose))

    def test_control_surfaces_are_cut_free(self):
        for surface, parent in [
            ("aileron_right", "wing"),
            ("aileron_left", "wing"),
            ("elevator_right", "horizontal_tail"),
            ("elevator_left", "horizontal_tail"),
            ("rudder", "fin"),
        ]:
            with self.subTest(surface=surface):
                self.assertAlmostEqual(overlap(self.parts[surface], self.parts[parent]), 0, delta=1e-6)

    def test_prop_discs_clear_the_airframe_while_tilting(self):
        # Grown by PROP_GAP: the tips keep that much room
        for name, (nose, _) in MOTORS.items():
            for tilt in tilts(name):
                hub = Location(hinge(nose)) * Rot(0, tilt, 0) * Pos(HUB_X, 0, -HINGE_UP)
                disc = hub * Rot(0, 90, 0) * Cylinder(PROP_RADIUS + PROP_GAP, 20)
                for label in AIRFRAME:
                    with self.subTest(motor=name, tilt=tilt, part=label):
                        self.assertAlmostEqual(overlap(disc, self.parts[label]), 0, delta=1e-6)

    def test_motors_clear_the_nacelles_and_hinges_while_tilting(self):
        fixed = [self.parts["nacelles"], self.parts["tail_nacelle"]]
        fixed += [hinge.location * part for hinge in self.parts["hinges"].children for part in hinge.children]
        for name, (nose, _) in MOTORS.items():
            for tilt in tilts(name):
                pose = Location(hinge(nose)) * Rot(0, tilt, 0)
                # The propeller is covered by the disc test above
                for part in (part for part in self.parts[name].children if part.label != "propeller"):
                    for other in fixed:
                        with self.subTest(motor=name, tilt=tilt, part=part.label, other=other.label):
                            self.assertAlmostEqual(overlap(pose * part, other), 0, delta=1e-6)

    def test_components_sit_inside_their_hosts(self):
        for label, (host, center, size) in components().items():
            # Grown by the skin on every side, so the printed wall stays whole around it
            grown = Pos(center) * Box(*(d + 2 * SKIN for d in size))
            with self.subTest(component=label):
                self.assertAlmostEqual(overlap(grown, self.parts[host]), grown.volume, delta=grown.volume * 1e-5)
                for other in (part for part in (*AIRFRAME, "gimbal_mount", "center_fitting") if part != host):
                    self.assertAlmostEqual(overlap(grown, self.parts[other]), 0, delta=1e-6, msg=other)

    def test_components_do_not_touch_each_other(self):
        boxes = {label: Pos(center) * Box(*size) for label, (_, center, size) in components().items()}
        labels = sorted(boxes)
        for i, a in enumerate(labels):
            for b in labels[i + 1:]:
                with self.subTest(a=a, b=b):
                    self.assertAlmostEqual(overlap(boxes[a], boxes[b]), 0, delta=1e-6)

    def test_spars_are_enclosed_by_the_airframe(self):
        for label, (start, end, radius) in spar_lines().items():
            rest = tube(start, end, radius)
            for part in AIRFRAME:
                if rest.solids():
                    rest -= self.parts[part]
            with self.subTest(spar=label):
                self.assertAlmostEqual(sum(solid.volume for solid in rest.solids()), 0, delta=1e-3)

    def test_skids_are_the_lowest_point(self):
        self.assertAlmostEqual(self.vtol.bounding_box().min.Z, GROUND_Z, delta=0.01)
        self.assertGreater(self.parts["gimbal"].bounding_box().min.Z, GROUND_Z + 20)


class FlightTest(unittest.TestCase):
    def test_loop_starts_and_ends_on_the_pad(self):
        for t in (0, fly.DURATION):
            with self.subTest(t=t):
                for value, start in zip(fly.position(t), (0, 0, 0)):
                    self.assertAlmostEqual(value, start, delta=1e-6)
                self.assertAlmostEqual(abs(fly.orientation(t)[3]), 1, delta=1e-6)
                self.assertEqual(fly.tilt(t), 90)

    def test_lap_is_one_full_turn(self):
        self.assertAlmostEqual(fly.lap_angle(fly.DURATION), 2 * pi)

    def test_rotation_keyframes_stay_under_180_degrees(self):
        # Quaternion tracks interpolate the shortest way, so larger steps would spin backwards
        self.assertLess(360 * fly.PROP_RPS / fly.FPS, 180)
        times = [i / fly.FPS for i in range(fly.DURATION * fly.FPS + 1)]
        yaw_steps = [abs(fly.lap_angle(b) - fly.lap_angle(a)) for a, b in zip(times, times[1:])]
        self.assertLess(max(yaw_steps), pi)


if __name__ == "__main__":
    unittest.main()
