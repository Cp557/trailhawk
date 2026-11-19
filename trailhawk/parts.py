"""Real parts the VTOL is built from: the single source for names, links, masses, sizes and the
specs that vtol.py, balance.py and performance.py use. Every number comes from the linked page,
checked on CHECKED; prices and stock change, so re-check before buying.

Sizes are the datasheet's (length, width, height) in mm unless a field says otherwise.
"""

CHECKED = "2026-11-05"  # payload and battery: 2026-11-09

MOTOR = {
    "name": "T-Motor AT2820 Long Shaft KV880",
    "url": "https://store.tmotor.com/product/at2820-long-shaft-fixed-wing-motor.html",
    "grams": 139,  # including cable
    "diameter": 35.2,
    "length": 63.5,  # "Motor Dimensions Φ35.2*63.5mm": body and long shaft together; the split is not published
    "shaft": 5,
    "kv": 880,
    "idle_amps": 1.6,  # at 10 V
    "resistance": 0.039,  # ohm
    "peak_amps": 45,  # for 180 s; no continuous rating is published
    "peak_watts": 700,  # for 180 s
    # Manufacturer static test, 4S, prop "APC 12*6":
    # (throttle %, volts, amps, watts, rpm, torque N·m, thrust g)
    "test": [
        (40, 15.15, 8.88, 134.43, 5738, 0.157, 946),
        (45, 15.12, 10.58, 159.95, 6114, 0.176, 1077),
        (50, 15.09, 12.29, 185.48, 6461, 0.195, 1194),
        (55, 15.06, 14.01, 211.03, 6734, 0.215, 1320),
        (60, 15.03, 15.95, 239.66, 7033, 0.242, 1437),
        (65, 14.98, 18.74, 280.67, 7422, 0.270, 1616),
        (70, 14.93, 21.84, 326.02, 7801, 0.303, 1793),
        (75, 14.87, 25.25, 375.36, 8141, 0.337, 1992),
        (80, 14.80, 29.60, 438.01, 8514, 0.382, 2230),
        (90, 14.66, 37.97, 556.43, 9174, 0.444, 2596),
        (100, 14.59, 42.00, 612.71, 9413, 0.472, 2755),
    ],
}

PROP = {
    "name": "APC 12x6E (LP12060E); the left motor takes the reverse-rotation 12x6EP (LP12060EP)",
    "url": "https://www.apcprop.com/product/12x6e/",
    "url_reverse": "https://www.apcprop.com/product/12x6ep/",
    "data_url": "https://www.apcprop.com/files/PER3_12x6E.dat",
    "grams": 27,  # 0.95 oz
    "diameter": 304.8,
    "pitch": 152.4,
    "hub_thickness": 10.4,
    "hub_diameter": 20.3,
    # APC's simulated performance (v2022-0915), "PROP RPM = 6000" block: (J, Ct, Cp)
    "data": [
        (0.0000, 0.0927, 0.0321),
        (0.0225, 0.0911, 0.0324),
        (0.0449, 0.0894, 0.0327),
        (0.0674, 0.0876, 0.0329),
        (0.0898, 0.0857, 0.0331),
        (0.1123, 0.0836, 0.0332),
        (0.1347, 0.0815, 0.0333),
        (0.1572, 0.0791, 0.0333),
        (0.1796, 0.0766, 0.0333),
        (0.2021, 0.0740, 0.0331),
        (0.2245, 0.0712, 0.0329),
        (0.2470, 0.0683, 0.0325),
        (0.2694, 0.0652, 0.0321),
        (0.2919, 0.0620, 0.0315),
        (0.3144, 0.0587, 0.0308),
        (0.3368, 0.0552, 0.0300),
        (0.3593, 0.0515, 0.0291),
        (0.3817, 0.0478, 0.0280),
        (0.4042, 0.0439, 0.0269),
        (0.4266, 0.0399, 0.0256),
        (0.4491, 0.0359, 0.0242),
        (0.4715, 0.0318, 0.0227),
        (0.4940, 0.0276, 0.0211),
        (0.5164, 0.0235, 0.0193),
        (0.5389, 0.0192, 0.0175),
        (0.5613, 0.0150, 0.0156),
        (0.5838, 0.0108, 0.0136),
        (0.6063, 0.0066, 0.0116),
        (0.6287, 0.0025, 0.0095),
    ],
}

ESC = {
    "name": "Hobbywing XRotor Pro 50A",
    "url": "https://www.hobbywing.com/en/products/xrotor-pro-50a233",
    "grams": 56,
    "size": (48.0, 30.0, 15.5),
    "amps": 50,  # continuous; 70 A peak
    "cells": (4, 6),
}

BATTERY = {
    "name": "Flywoo Explorer 4S2P 9000 mAh Li-ion (Molicel P45B 21700 cells), XT60",
    "url": "https://www.unmannedtechshop.co.uk/products/flywoo-explorer-molicell-p45b-21700-4s2p-9000mah-lionpack-battery",
    "grams": 573.5,
    # Not published. Estimate: 8 cells (dia 21.55 x 70.15 mm) in a 2x2 section, two cells long, plus wrap.
    "size": (145, 45, 45),
    "amp_hours": 9.0,
    "cells": 4,
    "cell_volts": 3.6,  # P45B cell nominal; the listing rounds the pack to 14.8 V
    "max_amps": 90,  # 2 x 45 A from the cell data; the listing's "45A" reads as the per-cell figure, so confirm it
}

FLIGHT_CONTROLLER = {
    "name": "Holybro Kakute H743-Wing (ArduPilot target KakuteH7-Wing)",
    "url": "https://holybro.com/products/kakute-h743-wing",
    "grams": 28,  # with USB extender
    "size": (45, 30, 13.5),
    "current_amps": 90,  # current sensor, continuous; 220 A peak
    # Servo rail: Holybro lists 6 V default / 8 V by jumper, ArduPilot's page 6 / 7.2 V. Plan on the lower.
    "servo_volts": 7.2,
}

AIRSPEED = {
    "name": "Matek ASPD-4525 digital airspeed sensor (TE 4525DO, I2C), with pitot tube and silicone tubing",
    "url": "https://www.mateksys.com/?p=7212",
    "grams": 3.5,  # sensor board; the pitot tube is an estimate in balance.GRAMS
    "size": (26, 16, 8),  # board size unpublished: estimate
}

GPS = {
    "name": "Matek M10Q-5883 (u-blox M10 GPS, QMC5883L compass)",
    "url": "https://www.mateksys.com/?portfolio=m10q-5883",
    "grams": 8,
    "size": (20, 20, 12.4),
}

RECEIVER = {
    "name": "RadioMaster RP3 V2 ExpressLRS 2.4 GHz",
    "url": "https://radiomasterrc.com/products/rp3-expresslrs-2-4ghz-nano-receiver",
    "grams": 4.6,
    "size": (22, 13, 4),
}

TILT_SERVO = {
    "name": "KST X12-708 (brushless, steel gears)",
    "url": "https://www.buddyrc.com/products/kst-x12-708-brushless-micro-servo-9-3kg-cm-0-07sec-60degree",
    "grams": 20,
    "size": (23.0, 12.0, 27.5),  # body; the spline position is not published (assumed 6 mm from one end)
    "torque": [(6.0, 6.3), (7.4, 8.6), (8.4, 9.3)],  # (volts, kgf·cm stall)
    "travel": 100,  # deg, per servodatabase.com
}

AILERON_SERVO = {
    "name": "KST X08 Plus",
    "url": "https://www.rotorama.com/product/kst-x08-plus-9g-0-09s-5-3kg",
    "grams": 9,
    "size": (23.5, 8.0, 16.8),
}

TAIL_SERVO = {
    "name": "KST X06 V6.0",
    "url": "https://www.rotorama.com/product/kst-x06-6g-0-07s-1-8kg",
    "grams": 6,
    "size": (20.0, 7.0, 16.6),
}

# Payload. The AI HAT+ stacks on the Pi; the height is an estimate (board, standoffs, HAT, cooler).
COMPUTER = {
    "name": "Raspberry Pi 5 with Raspberry Pi AI HAT+ (13 TOPS) and active cooler",
    "url": "https://www.raspberrypi.com/products/ai-hat/",
    "grams": 46 + 16 + 20,  # board (estimate, unpublished) + HAT (reseller) + cooler (estimate)
    "size": (85, 56, 32),
    "watts": 10,  # estimate under load: neither maker publishes a loaded figure
}

# Upgrade path: a Boson 320 with the 6.3 mm, 34 deg lens (https://www.oemcameras.com/products/20320a034,
# $2,131 on 2026-11-09, 7.5 g + lens, 21 x 21 x 11 mm without lens) fits the same turret and resolves ~3.5x finer.
THERMAL_CAMERA = {
    "name": "Teledyne FLIR Lepton 3.5 (160 x 120, radiometric) on a GroupGets PureThermal 3 USB board",
    "url": "https://groupgets.com/products/flir-lepton-3-5",  # $164.00; the board is $119.99, checked 2026-11-10
    "url_board": "https://www.sparkfun.com/purethermal-3-flir-lepton-smart-i-o-board.html",
    "grams": 6,  # estimate: neither the module nor the board publishes a weight
    "size": (29, 26, 12),  # board 25.8 x 28.9 mm (listing); the height with the module is an estimate
    "pixels": 160,
    "hfov": 57,  # deg
    "watts": 0.5,  # estimate: a similar Lepton USB board draws 54 mA at 5 V
}

COLOR_CAMERA = {
    "name": "Raspberry Pi Camera Module 3 (standard)",
    "url": "https://www.raspberrypi.com/documentation/accessories/camera.html",
    "grams": 4,
    "size": (25, 24, 11.5),
    "hfov": 66,  # deg
    "watts": 1,  # estimate
}
