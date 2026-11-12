"""Power and flight time of the VTOL from its real parts (parts.py): APC's coefficients for the
12x6E prop, T-Motor's measured 4S test of the AT2820 on that prop for the motor and ESC losses,
a drag buildup (Raymer) for the airframe, the battery's capacity and the tilt servo's torque.

Hover lifts on all three motors; forward flight runs on the two front motors with the rear
motor stopped (ArduPilot tilt-tricopter). Everything takes the air it flies in, an `Air` from
balance.py: the drone is sized for DESIGN_AIR (12,000 ft) and the propulsion data is at sea level.
Run this file for a report.
"""

from functools import cache
from math import acos, log10, pi, sqrt

import numpy as np

from balance import DESIGN_AIR, G, GRAMS, SEA_LEVEL, Air, hover_shares, mass_items, mean_aerodynamic_chord, stall_speed
from parts import BATTERY, COLOR_CAMERA, COMPUTER, FLIGHT_CONTROLLER, MOTOR, PROP, THERMAL_CAMERA, TILT_SERVO
from vtol import (
    CAMERA_EXPOSED, CAMERA_RADIUS, CAN_LENGTH, FUSELAGE, HINGE_UP, MOTORS, NACELLE, POD_RADIUS, PROP_RADIUS,
    SHAFT_OUT, STRUT_CHORD, TILTING, YAW_TILT, build_vtol, fin_stations, tail_stations, wing_stations,
)  # fmt: skip

OSWALD = 0.8  # span efficiency (winglets not credited)
LEAKAGE = 1.1  # parasite drag allowance for interference, gaps and unmodeled bumps
USABLE = 0.8  # land with 20% charge left
SAG_CELL_VOLTS = 3.2  # Li-ion loaded near the end of a flight
ELECTRONICS_WATTS = COMPUTER["watts"] + THERMAL_CAMERA["watts"] + COLOR_CAMERA["watts"] + 3  # payload, plus 3 W for avionics and servos (estimate)
THRUST_MARGIN = 1.6  # max static thrust each motor needs over its hover share, for control
SERVO_MARGIN = 1.5  # tilt servo stall torque over its worst steady load
SPEED_MARGIN = 1.3  # slowest cruise speed over stall
HOVER_SECONDS = 2 * 30 + 2 * 15  # takeoff and landing, two transitions; all at hover power
ORBIT_RADIUS = 75  # m, circling the hiker

DIAMETER = PROP["diameter"] / 1000  # m
ADVANCE, THRUST_COEFFICIENT, POWER_COEFFICIENT = (np.array(column) for column in zip(*PROP["data"]))


def prop_thrust(rpm, speed, air):
    """Thrust (N) of one prop at `rpm` and airspeed `speed` m/s, from APC's coefficients."""
    n = rpm / 60
    return np.interp(speed / (n * DIAMETER), ADVANCE, THRUST_COEFFICIENT, right=0) * air.density * n**2 * DIAMETER**4


def prop_power(rpm, speed, air):
    """Shaft power (W) of one prop at `rpm` and airspeed `speed` m/s."""
    n = rpm / 60
    return np.interp(speed / (n * DIAMETER), ADVANCE, POWER_COEFFICIENT) * air.density * n**3 * DIAMETER**5


def rpm_for_thrust(thrust, speed, air):
    """RPM at which one prop makes `thrust` N at `speed` m/s (bisection)."""
    low = 60 * speed / (ADVANCE[-1] * DIAMETER) if speed else 0.0  # thrust ~0 at the table's last J
    high = 30000.0
    for _ in range(60):
        middle = (low + high) / 2
        low, high = (middle, high) if prop_thrust(middle, speed, air) < thrust else (low, middle)
    return (low + high) / 2


def motor_test():
    """The manufacturer's static test as arrays: throttle %, volts, amps, battery W, rpm, shaft W, thrust N."""
    throttle, volts, amps, watts, rpm, torque, grams = (np.array(column) for column in zip(*MOTOR["test"]))
    return throttle, volts, amps, watts, rpm, torque * rpm * 2 * pi / 60, grams / 1000 * G


def battery_power(shaft):
    """Battery power (W) to deliver `shaft` W: motor and ESC efficiency interpolated from the
    manufacturer's test (held at the end values outside the tested range)."""
    _, _, _, watts, _, shaft_tested, _ = motor_test()
    return shaft / np.interp(shaft, shaft_tested, shaft_tested / watts)


def max_static(volts, air):
    """(thrust N, current A) of one motor at full throttle on `volts` in `air`.

    The test's 100% row, scaled by voltage squared (rpm ~ V, so thrust and current ~ V^2). In
    thinner air the prop loads the motor less and it spins up, along a straight torque-speed line
    from the tested point to no-load, until the motor's torque matches the prop's (~ density rpm^2).
    Thrust and current both scale with that torque."""
    _, tested_volts, amps, _, rpm, _, thrust = motor_test()
    voltage = (volts / tested_volts[-1]) ** 2
    sigma = air.density / SEA_LEVEL.density
    no_load = MOTOR["kv"] * tested_volts[-1] / rpm[-1]  # relative to the tested rpm
    speed_up = (sqrt(1 + 4 * sigma * (no_load - 1) * no_load) - 1) / (2 * sigma * (no_load - 1))
    torque = sigma * speed_up**2
    return thrust[-1] * voltage * torque, amps[-1] * voltage * torque


@cache
def weight(vtol):
    """All-up weight in N."""
    return sum(grams for _, grams, _ in mass_items(vtol)) / 1000 * G


@cache
def hover(vtol, air):
    """Per motor: (thrust N, rpm, battery W) in a level hover, from hover_shares."""
    result = {}
    for name, share in hover_shares(vtol).items():
        thrust = share * weight(vtol)
        rpm = rpm_for_thrust(thrust, 0, air)
        result[name] = (thrust, rpm, battery_power(prop_power(rpm, 0, air)))
    return result


def hover_need(vtol):
    """Full-throttle thrust (N) the busiest motor needs: THRUST_MARGIN over its hover share."""
    return THRUST_MARGIN * max(hover_shares(vtol).values()) * weight(vtol)


def thinnest_air(vtol):
    """Lowest air density (kg/m^3) where full throttle on a sagging battery still meets
    hover_need (bisection: thrust falls as the air thins)."""
    volts = BATTERY["cells"] * SAG_CELL_VOLTS
    low, high = 0.3, SEA_LEVEL.density
    for _ in range(50):
        middle = (low + high) / 2
        low, high = (middle, high) if max_static(volts, Air(middle, SEA_LEVEL.viscosity))[0] < hover_need(vtol) else (low, middle)
    return high


def heaviest_takeoff(vtol, air):
    """All-up weight (g) at which full throttle on a sagging battery just meets hover_need in `air`."""
    thrust, _ = max_static(BATTERY["cells"] * SAG_CELL_VOLTS, air)
    return thrust / hover_need(vtol) * weight(vtol) / G * 1000


def hottest_day(density, feet):
    """Hottest temperature (C) at pressure altitude `feet` (standard-atmosphere pressure) at which
    the air is still at least `density`."""
    pressure = 101325 * (1 - 2.25577e-5 * feet * 0.3048) ** 5.25588
    return pressure / (287.05 * density) - 273.15


def tilt_servo_load(vtol):
    """Worst steady torque on a front tilt servo (kgf·cm): THRUST_MARGIN times its hover thrust
    plus the motor unit's weight, both taken at HINGE_UP from the hinge."""
    thrust = max(hover(vtol, SEA_LEVEL)[name][0] for name in TILTING) * THRUST_MARGIN / G
    unit = sum(GRAMS[part] for part in ("motor", "propeller", "prop_nut", "mount")) / 1000
    return (thrust + unit) * HINGE_UP / 10


def tilt_servo_torque():
    """Stall torque (kgf·cm) of the tilt servo at the flight controller's servo rail voltage."""
    volts, torque = zip(*TILT_SERVO["torque"])
    return float(np.interp(FLIGHT_CONTROLLER["servo_volts"], volts, torque))


def skin_friction(reynolds):
    """Turbulent flat-plate skin-friction coefficient: conservative at these Reynolds numbers."""
    return 0.455 / log10(reynolds) ** 2.58


def airfoil_form_factor(thickness, max_at=0.3):
    return 1 + 0.6 / max_at * thickness + 100 * thickness**4


def body_form_factor(length, diameter):
    fineness = length / diameter
    return 1 + 60 / fineness**3 + fineness / 400


def nacelle_form_factor(length, diameter):
    return 1 + 0.35 / (length / diameter)


@cache
def wetted_components(vtol):
    """(name, wetted area m^2, reference length m, form factor) for the skin-friction buildup.

    Wetted areas are the CAD surface areas, including the bits buried in the fuselage or wing,
    which overstates skin friction by a few percent."""
    parts = {part.label: part for part in vtol.children}

    def area(*labels):
        return sum(parts[label].area for label in labels) / 1e6

    def mean_chord(stations):
        return sum(chord for _, chord, _ in stations) / len(stations) / 1000

    length = FUSELAGE[-1][0] / 1000
    diameter = max(2 * sqrt(w * h) for _, w, h, _ in FUSELAGE) / 1000
    nacelle_length = sum(run for _, run in MOTORS.values()) / len(MOTORS) / 1000
    nacelle_diameter = 2 * max(r for _, r in NACELLE) / 1000
    return [
        ("wing", area("wing", "aileron_right", "aileron_left"), mean_aerodynamic_chord()[0] / 1000, airfoil_form_factor(0.12)),
        ("horizontal tail", area("horizontal_tail", "elevator_right", "elevator_left"), mean_chord(tail_stations()), airfoil_form_factor(0.10)),
        ("fin", area("fin", "rudder"), mean_chord(fin_stations()), airfoil_form_factor(0.10)),
        ("fuselage", area("fuselage"), length, body_form_factor(length, diameter)),
        ("nacelles", area("nacelles", "tail_nacelle"), nacelle_length, nacelle_form_factor(nacelle_length, nacelle_diameter)),
        ("landing gear", area("landing_gear"), STRUT_CHORD / 1000, airfoil_form_factor(0.25)),  # faired: struts and rails
    ]


@cache
def protrusions(vtol):
    """(name, drag area CD * frontal area in m^2) for the bluff parts in the airstream."""
    hinges = next(part for part in vtol.children if part.label == "hinges").children
    servos = [part.bounding_box() for hinge in hinges for part in hinge.children if part.label == "tilt_servo"]
    rear_motor = MOTOR["diameter"] * CAN_LENGTH + MOTOR["shaft"] * SHAFT_OUT
    r, h = CAMERA_RADIUS, CAMERA_EXPOSED
    camera = r**2 * acos(1 - h / r) - (r - h) * sqrt(2 * r * h - h**2)  # the circular segment below the belly
    return [
        ("camera ball", 0.47 * camera / 1e6),  # sphere, subcritical; only the part below the belly
        ("tilt servos", 1.0 * sum(box.size.Y * (box.max.Z - (POD_RADIUS - HINGE_UP)) for box in servos) / 1e6),  # above the nacelles
        ("rear motor, upright", 0.8 * rear_motor / 1e6),  # short cylinder in crossflow
        ("rear prop, stopped", 1.0 * 2 * PROP_RADIUS * 6 / 1e6),  # blades edge-on, ~6 mm projected (estimate)
    ]


def parasite_drag_area(vtol, speed, air):
    """Equivalent flat-plate drag area f = CD0 * S (m^2) at `speed` m/s, with its breakdown."""
    breakdown = [
        (name, skin_friction(speed * length / air.viscosity) * form * wetted)
        for name, wetted, length, form in wetted_components(vtol)
    ] + protrusions(vtol)
    return LEAKAGE * sum(f for _, f in breakdown), breakdown


def wing_reference():
    """(area m^2, aspect ratio): trapezoid panels to the winglet roots."""
    _, _, area = mean_aerodynamic_chord()
    span = 2 * wing_stations()[-1][0].origin.Y
    return area / 1e6, span**2 / area


def cruise(vtol, speed, air, load_factor=1):
    """Level flight at `speed` m/s on the front motors, propulsion only: (battery W, lift
    coefficient, lift/drag, rpm). `load_factor` > 1 is a banked turn."""
    area, aspect = wing_reference()
    dynamic = 0.5 * air.density * speed**2
    lift = load_factor * weight(vtol) / (dynamic * area)
    drag = (parasite_drag_area(vtol, speed, air)[0] + lift**2 / (pi * OSWALD * aspect) * area) * dynamic
    rpm = rpm_for_thrust(drag / len(TILTING), speed, air)
    return len(TILTING) * battery_power(prop_power(rpm, speed, air)), lift, weight(vtol) / drag, rpm


def speeds(vtol, air):
    """Cruise speeds considered: SPEED_MARGIN over stall up to 25 m/s, in 0.5 m/s steps."""
    stall = stall_speed(weight(vtol) / G * 1000, air)
    return np.arange(np.ceil(SPEED_MARGIN * stall * 2) / 2, 25.01, 0.5)


def orbit_load(speed):
    """Load factor of a level turn at `speed` m/s around ORBIT_RADIUS."""
    return sqrt(1 + (speed**2 / (G * ORBIT_RADIUS)) ** 2)


def mission(vtol, air):
    """Takeoff, transition, then either cruise at the best-range speed or circle at the best
    loiter speed, transition, landing, on USABLE charge. Watts include the electronics."""
    hover_watts = sum(watts for _, _, watts in hover(vtol, air).values()) + ELECTRONICS_WATTS
    straight = {float(v): cruise(vtol, v, air)[0] + ELECTRONICS_WATTS for v in speeds(vtol, air)}
    orbit = {float(v): cruise(vtol, v, air, orbit_load(v))[0] + ELECTRONICS_WATTS for v in speeds(vtol, air)}
    endurance_speed = min(straight, key=straight.get)
    range_speed = min(straight, key=lambda v: straight[v] / v)
    loiter_speed = min(orbit, key=orbit.get)
    energy = BATTERY["cells"] * BATTERY["cell_volts"] * BATTERY["amp_hours"] * USABLE * 3600  # J
    airborne = energy - hover_watts * HOVER_SECONDS
    return {
        "hover_watts": hover_watts,
        "hover_minutes": energy / hover_watts / 60,
        "endurance_speed": endurance_speed,
        "endurance_watts": straight[endurance_speed],
        "range_speed": range_speed,
        "range_watts": straight[range_speed],
        "cruise_minutes": airborne / straight[range_speed] / 60,
        "range_km": airborne / straight[range_speed] * range_speed / 1000,
        "loiter_speed": loiter_speed,
        "loiter_watts": orbit[loiter_speed],
        "loiter_minutes": airborne / orbit[loiter_speed] / 60,
        "energy": energy,
    }


def report_conditions(vtol, title, air):
    """Hover, thrust margin, drag and mission in one `air`."""
    nominal, sagged = BATTERY["cells"] * BATTERY["cell_volts"], BATTERY["cells"] * SAG_CELL_VOLTS
    results = mission(vtol, air)
    print(f"\n{title} (density {air.density} kg/m^3)")
    for name, (thrust, rpm, watts) in hover(vtol, air).items():
        print(f"  hover {name:14s} {thrust / G * 1000:5.0f} g  {rpm:5.0f} rpm  {watts:5.0f} W")
    print(f"  hover total    {results['hover_watts']:5.0f} W with electronics, {results['hover_watts'] / nominal:.1f} A at {nominal:.1f} V, {results['hover_minutes']:.1f} min on {USABLE:.0%} of {nominal * BATTERY['amp_hours']:.0f} Wh")
    thrust, amps = max_static(sagged, air)
    need = hover_need(vtol)
    print(f"  full throttle  {thrust / G * 1000:5.0f} g per motor at {sagged:.1f} V (needs {need / G * 1000:.0f} g), {amps:.1f} A; all three {len(MOTORS) * amps:.0f} A vs battery {BATTERY['max_amps']} A")
    speed = results["range_speed"]
    area, aspect = wing_reference()
    total, breakdown = parasite_drag_area(vtol, speed, air)
    print(f"  CD0 {total / area:.4f} at {speed:.1f} m/s (wing {area:.3f} m^2, aspect ratio {aspect:.1f}): " + ", ".join(f"{name} {f / (total / LEAKAGE):.0%}" for name, f in breakdown))
    print("  speed  CL    L/D   rpm   power (propulsion + electronics)")
    for v in speeds(vtol, air)[::2]:
        watts, lift, ratio, rpm = cruise(vtol, v, air)
        print(f"  {v:5.1f}  {lift:.2f}  {ratio:4.1f}  {rpm:5.0f}  {watts:4.0f} + {ELECTRONICS_WATTS} W")
    print(f"  best endurance {results['endurance_speed']:.1f} m/s at {results['endurance_watts']:.0f} W; best range {results['range_speed']:.1f} m/s at {results['range_watts']:.0f} W")
    print(f"  circling r={ORBIT_RADIUS} m  {results['loiter_speed']:.1f} m/s at {results['loiter_watts']:.0f} W: {results['loiter_minutes']:.0f} min after {HOVER_SECONDS} s at hover power")
    print(f"  cruising       {results['cruise_minutes']:.0f} min, {results['range_km']:.1f} km")


def report(vtol):
    print(f"PROPULSION CHECK: APC data vs {MOTOR['name']} test (4S, APC 12x6, sea level)")
    for throttle, _, _, _, rpm, shaft, thrust in zip(*motor_test()):
        print(f"  {throttle:3.0f}%  {rpm:5.0f} rpm  thrust {prop_thrust(rpm, 0, SEA_LEVEL) / thrust - 1:+5.1%}  shaft power {prop_power(rpm, 0, SEA_LEVEL) / shaft - 1:+5.1%}")
    report_conditions(vtol, "12,000 ft, 10 C (design)", DESIGN_AIR)
    report_conditions(vtol, "Sea level", SEA_LEVEL)
    limit = thinnest_air(vtol)
    print(f"\nHover limit ({THRUST_MARGIN}x thrust margin on a sagging battery): air down to {limit:.3f} kg/m^3")
    print("  hottest day   " + ", ".join(f"{feet:,} ft {hottest_day(limit, feet):.0f} C" for feet in (10000, 12000, 13000, 14000)))
    print(f"  heaviest takeoff at 12,000 ft, 10 C: {heaviest_takeoff(vtol, DESIGN_AIR):.0f} g (now {weight(vtol) / G * 1000:.0f} g)")
    print(f"\nTilt servo {tilt_servo_torque():.1f} kgf-cm at {FLIGHT_CONTROLLER['servo_volts']} V vs {tilt_servo_load(vtol):.1f} kgf-cm load; front motors tilt 0-{90 + YAW_TILT} deg, servo travel {TILT_SERVO['travel']} deg")


if __name__ == "__main__":
    report(build_vtol())
