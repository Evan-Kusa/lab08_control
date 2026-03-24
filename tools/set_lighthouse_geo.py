#!/usr/bin/env python3
"""Write Lighthouse V2 base station geometry directly to a Crazyflie.

Adjust bs1_pos / bs2_pos below to match your physical setup.
Coordinate frame: +X toward base stations, +Y left, +Z up.

Usage:
    python3 set_lighthouse_geo.py --drone 2
    python3 set_lighthouse_geo.py --uri radio://0/80/2M/E7E7E7E7D3
"""

import numpy as np
import time
from cflib.crazyflie.mem import LighthouseBsGeometry, LighthouseMemHelper
from cf_utils import parse_uri_args, connect_and_wait

# ── Adjust these to your setup (meters) ──────────────────────────
# BS1: 190cm forward, 120cm LEFT, 170cm high
bs1_pos = np.array([1.90, 1.20, 1.70])
# BS2: 190cm forward, 120cm RIGHT, 170cm high
bs2_pos = np.array([1.90, -1.20, 1.70])
# ──────────────────────────────────────────────────────────────────


def look_at_rotation(bs_pos, target=np.array([0.0, 0.0, 0.0])):
    """Rotation matrix for a base station looking at target.
    Convention: BS local Z is the light-emission (look) direction."""
    forward = target - bs_pos
    forward /= np.linalg.norm(forward)

    z_axis = forward  # BS Z points toward target (emission direction)
    world_up = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(world_up, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)

    return np.column_stack([x_axis, y_axis, z_axis])


def main():
    uri = parse_uri_args("Write Lighthouse geometry to a Crazyflie")

    R1 = look_at_rotation(bs1_pos)
    R2 = look_at_rotation(bs2_pos)

    print(f"BS1  pos: {bs1_pos}")
    print(f"BS1  rot:\n{R1}\n")
    print(f"BS2  pos: {bs2_pos}")
    print(f"BS2  rot:\n{R2}\n")

    cf = connect_and_wait(uri)
    if cf is None:
        return

    geo0 = LighthouseBsGeometry()
    geo0.origin = bs1_pos.tolist()
    geo0.rotation_matrix = R1.tolist()
    geo0.valid = True

    geo1 = LighthouseBsGeometry()
    geo1.origin = bs2_pos.tolist()
    geo1.rotation_matrix = R2.tolist()
    geo1.valid = True

    write_done = False

    def write_complete(success):
        nonlocal write_done
        write_done = True
        print(f"Geometry write {'succeeded' if success else 'FAILED'}!")

    helper = LighthouseMemHelper(cf)
    helper.write_geos({0: geo0, 1: geo1}, write_complete)

    for _ in range(100):
        if write_done:
            break
        time.sleep(0.1)

    cf.close_link()
    print("Done!")


if __name__ == "__main__":
    main()
