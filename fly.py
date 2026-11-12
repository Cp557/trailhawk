"""Animate the VTOL in the OCP CAD Viewer.

Start the viewer first (`python -m ocp_viewer`), then run `python fly.py`. The 27 s loop:
spool up on the pad, vertical climb, transition to wing-borne flight, one banked left lap,
transition back to hover over the pad, descent and landing. The front motors tilt; the fixed
rear motor runs only while hovering.

Viewer rotation tracks are quaternion keyframes (shortest-path interpolation), so every
rotating track is sampled densely enough that consecutive keyframes differ by < 180 deg.
"""

from math import cos, pi, radians, sin

from build123d import Align, Cylinder, Pos
from ocp_vscode import Animation, set_port, show

from drone import labeled
from vtol import GROUND_Z, MOTORS, TILTING, build_vtol

FPS = 10  # keyframes per second
DURATION = 27
ALTITUDE = 1500
RADIUS = 2000  # lap radius; the nose flies a left circle starting and ending over the pad
LAP_RATE = 2 * pi / 15  # rad/s in cruise; the accel, cruise and decel phases add up to one lap
PROP_RPS = 4  # visual spin: 144 deg per keyframe


def ramp(t, t0, t1):
    """Smoothstep from 0 at t0 to 1 at t1."""
    u = min(max((t - t0) / (t1 - t0), 0), 1)
    return u * u * (3 - 2 * u)


def ramp_integral(t, t0, t1):
    """Integral of ramp(t0, t1) from 0 to t."""
    if t <= t0:
        return 0
    if t >= t1:
        return t - t1 + (t1 - t0) / 2
    u = (t - t0) / (t1 - t0)
    return (u**3 - u**4 / 2) * (t1 - t0)


def bump(t, t0, t1):
    """0 -> 1 -> 0 between t0 and t1."""
    return sin(pi * (t - t0) / (t1 - t0)) if t0 < t < t1 else 0


def lap_angle(t):
    """Heading change around the lap: speed ramps up over 5-8 s, holds, ramps down over 20-23 s."""
    if t < 5:
        return 0
    if t < 8:
        return LAP_RATE * (t - 5) ** 2 / 6
    if t < 20:
        return LAP_RATE * (1.5 + t - 8)
    if t < 23:
        return LAP_RATE * (13.5 + (t - 20) - (t - 20) ** 2 / 6)
    return 2 * pi


def position(t):
    phi = lap_angle(t)
    altitude = ALTITUDE * (ramp(t, 2, 5) - ramp(t, 23, 26))
    return (-RADIUS * sin(phi), -RADIUS * (1 - cos(phi)), altitude)


def orientation(t):
    """Quaternion (x, y, z, w): yaw along the lap applied after a left bank (body roll about X)."""
    yaw = lap_angle(t)
    bank = radians(25) * (ramp(t, 8, 9) - ramp(t, 19, 20))
    cz, sz, cx, sx = cos(yaw / 2), sin(yaw / 2), cos(bank / 2), sin(bank / 2)
    return (cz * sx, sz * sx, sz * cx, cz * cx)


def tilt(t):
    """Motor tilt: 90 = hover (props up), 0 = cruise (props forward)."""
    return 90 * (1 - ramp(t, 5, 8) + ramp(t, 20, 23))


def rear_spin(t):
    """Rear prop angle (deg): full speed while hovering, stopped while the front motors fly the
    wing; its spin rate follows tilt(t) / 90."""
    return 360 * PROP_RPS * (t - ramp_integral(t, 5, 8) + ramp_integral(t, 20, 23))


def aileron(t):
    """Right aileron trailing edge down rolls into the left bank, then the reverse to roll out."""
    return 15 * (bump(t, 8, 9) - bump(t, 19, 20))


def elevator(t):
    """Right elevator: trailing edge up through the transition and the banked turn."""
    return -10 * bump(t, 5, 8) - 6 * (ramp(t, 8, 9) - ramp(t, 19, 20))


def rudder(t):
    return 8 * (ramp(t, 8, 9) - ramp(t, 19, 20))


def gimbal_pan(t):
    """Camera swings to look into the turn (toward the pad) during the lap."""
    return 90 * (ramp(t, 9, 11) - ramp(t, 17, 19))


def fly():
    pad = Pos(0, -RADIUS, GROUND_Z) * Cylinder(RADIUS + 1500, 10, align=(Align.CENTER, Align.CENTER, Align.MAX))
    set_port(3939)  # default port of both the VS Code extension and `python -m ocp_viewer`
    show(build_vtol(), labeled(pad, "ground", "darkseagreen"), reset_camera="reset")

    times = [i / FPS for i in range(DURATION * FPS + 1)]
    animation = Animation()

    def track(path, action, value):
        animation.add_track(f"/Group/vtol{path}", action, times, [value(t) for t in times])

    track("", "t", position)
    track("", "q", orientation)
    for name in TILTING:
        spin = -1 if name == "motor_left" else 1  # counter-rotate the left prop
        track(f"/{name}", "ry", tilt)
        track(f"/{name}/propeller", "rx", lambda t, spin=spin: spin * 360 * PROP_RPS * t)
    for name in set(MOTORS) - set(TILTING):
        track(f"/{name}/propeller", "rx", rear_spin)
    # Both ailerons' hinge axes point outboard, so equal angles give opposite deflections (roll);
    # the elevators need opposite angles to move together.
    track("/aileron_right", "rx", aileron)
    track("/aileron_left", "rx", aileron)
    track("/elevator_right", "rx", elevator)
    track("/elevator_left", "rx", lambda t: -elevator(t))
    track("/rudder", "rx", rudder)
    track("/gimbal", "rz", gimbal_pan)
    animation.animate(speed=1)


if __name__ == "__main__":
    fly()
