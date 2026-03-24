#!/usr/bin/env python3
"""Restore saved Lighthouse V2 base station geometry to the Crazyflie.
Run this after a battery swap to skip recalibration.

Usage:
    python3 restore_lighthouse_geo.py --drone 2
    python3 restore_lighthouse_geo.py --uri radio://0/80/2M/E7E7E7E7D3
"""

import time
from cflib.crazyflie.mem import LighthouseBsGeometry, LighthouseMemHelper
from cf_utils import parse_uri_args, connect_and_wait, reset_kalman

# ── Kabsch-corrected geometry (2026-03-24) ────────────────────────
# Original cfclient values rotated to align world frame:
# +X toward base stations, +Y left, +Z up
GEOMETRY = {
    0: {
        "origin": [1.9398418276905405, 1.2730497814824846, 1.7356479510915366],
        "rotation": [
            [-0.5826927971429376, 0.7418038342349602, -0.3319581621902921],
            [-0.6501613875755498, -0.6705731851508071, -0.35724192482870876],
            [-0.4876056868675091, 0.007664081680995009, 0.8730303447471098],
        ],
    },
    1: {
        "origin": [1.8563430628708077, -1.220313719491074, 1.660938529937039],
        "rotation": [
            [-0.6635100254928767, -0.6592734880411962, -0.35371299009892926],
            [0.5443897364392298, -0.7497346900349782, 0.3762150087668364],
            [-0.5132194575661866, 0.05706470970335016, 0.8563582030216256],
        ],
    },
}
# ──────────────────────────────────────────────────────────────────


def main():
    uri = parse_uri_args("Restore saved Lighthouse geometry to a Crazyflie")
    cf = connect_and_wait(uri)
    if cf is None:
        return

    # Build geometry objects
    geos = {}
    for bs_id, data in GEOMETRY.items():
        geo = LighthouseBsGeometry()
        geo.origin = data["origin"]
        geo.rotation_matrix = data["rotation"]
        geo.valid = True
        geos[bs_id] = geo
        print(f"BS{bs_id+1}: {data['origin']}")

    # Write
    write_done = False

    def on_write(success):
        nonlocal write_done
        write_done = True
        print(f"Geometry write {'succeeded' if success else 'FAILED'}!")

    helper = LighthouseMemHelper(cf)
    helper.write_geos(geos, on_write)

    for _ in range(100):
        if write_done:
            break
        time.sleep(0.1)

    reset_kalman(cf)
    time.sleep(1)
    cf.close_link()
    print("Done! Ready to fly.")


if __name__ == "__main__":
    main()
