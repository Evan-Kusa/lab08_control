#!/usr/bin/env python3
"""Read Lighthouse geometry from the Crazyflie, apply a world-frame rotation
so that the base stations end up where we expect them, and write back.

The cfclient 2-step estimation gets relative geometry right but the
world frame can be rotated.  This script fixes that by computing the
optimal rotation (Kabsch) from the current BS positions to the desired
BS positions, then applying it to both positions and rotation matrices.

Usage:
    python3 fix_lighthouse_frame.py --drone 2
    python3 fix_lighthouse_frame.py --uri radio://0/80/2M/E7E7E7E7D3
"""

import numpy as np
import time
from cflib.crazyflie.mem import LighthouseBsGeometry, LighthouseMemHelper
from cf_utils import parse_uri_args, connect_and_wait

# ── Desired base station positions (meters) ──────────────────────
# +X toward base stations, +Y left, +Z up
DESIRED = {
    0: np.array([1.90,  1.20, 1.70]),   # BS1: forward, left, up
    1: np.array([1.90, -1.20, 1.70]),   # BS2: forward, right, up
}
# ──────────────────────────────────────────────────────────────────


def kabsch_rotation(P, Q):
    """Find rotation R that minimises ||R @ P - Q||.
    P, Q: (N, 3) arrays of corresponding points."""
    H = P.T @ Q
    U, _, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    S = np.diag([1, 1, d])  # ensure proper rotation
    return Vt.T @ S @ U.T


def main():
    uri = parse_uri_args("Fix Lighthouse world frame via Kabsch rotation")
    cf = connect_and_wait(uri)
    if cf is None:
        return

    # ── Read current geometry ──
    read_done = False
    geos_read = {}

    def on_geos_read(geos):
        nonlocal read_done, geos_read
        geos_read = geos
        read_done = True

    helper = LighthouseMemHelper(cf)
    helper.read_all_geos(on_geos_read)

    for _ in range(100):
        if read_done:
            break
        time.sleep(0.1)
    if not read_done:
        print("Failed to read geometry!")
        cf.close_link()
        return

    # Extract current positions and rotations
    cur_pos = {}
    cur_rot = {}
    for bs_id in [0, 1]:
        geo = geos_read[bs_id]
        cur_pos[bs_id] = np.array(geo.origin)
        cur_rot[bs_id] = np.array(geo.rotation_matrix)
        print(f"Current BS{bs_id+1} pos: {cur_pos[bs_id]}")
        print(f"Current BS{bs_id+1} rot:\n{cur_rot[bs_id]}\n")

    # ── Compute correction rotation ──
    P = np.array([cur_pos[0], cur_pos[1]])  # current
    Q = np.array([DESIRED[0], DESIRED[1]])   # desired
    R = kabsch_rotation(P, Q)

    print(f"Correction rotation:\n{R}\n")

    # Verify
    for bs_id in [0, 1]:
        corrected = R @ cur_pos[bs_id]
        print(f"BS{bs_id+1} corrected pos: {corrected}  (desired: {DESIRED[bs_id]})")

    # ── Apply rotation and write ──
    new_geos = {}
    for bs_id in [0, 1]:
        geo = LighthouseBsGeometry()
        geo.origin = (R @ cur_pos[bs_id]).tolist()
        geo.rotation_matrix = (R @ cur_rot[bs_id]).tolist()
        geo.valid = True
        new_geos[bs_id] = geo

    write_done = False

    def on_write(success):
        nonlocal write_done
        write_done = True
        print(f"\nGeometry write {'succeeded' if success else 'FAILED'}!")

    helper.write_geos(new_geos, on_write)

    for _ in range(100):
        if write_done:
            break
        time.sleep(0.1)

    cf.close_link()
    print("Done!")


if __name__ == "__main__":
    main()
