#!/usr/bin/env python3
"""Hard reboot the Crazyflie remotely (no power cycle needed).
Also restores Lighthouse geometry and resets the Kalman estimator.

Usage:
    python3 reset_drone.py --drone 2
    python3 reset_drone.py --uri radio://0/80/2M/E7E7E7E7D3
"""

import time
import sys
import cflib.crtp
from cflib.bootloader import Bootloader
from cflib.crazyflie.mem import LighthouseBsGeometry, LighthouseMemHelper
from cf_utils import parse_uri_args, connect_and_wait, reset_kalman

# Import geometry from restore script so there's one source of truth
from restore_lighthouse_geo import GEOMETRY


def main():
    uri = parse_uri_args("Hard reboot a Crazyflie and restore Lighthouse geometry")

    cflib.crtp.init_drivers()

    # Step 1: Reboot via bootloader
    print(f"Rebooting drone at {uri}...")
    bl = Bootloader(uri)
    bl.start_bootloader(warm_boot=True)
    time.sleep(1)
    bl.reset_to_firmware()
    bl.close()
    print("Reboot sent! Waiting for drone to come back up...")
    time.sleep(4)

    # Step 2: Reconnect
    cf = connect_and_wait(uri)
    if cf is None:
        print("Reboot succeeded but failed to reconnect. Try again or power cycle.",
              file=sys.stderr)
        sys.exit(1)

    # Step 3: Restore Lighthouse geometry
    geos = {}
    for bs_id, data in GEOMETRY.items():
        geo = LighthouseBsGeometry()
        geo.origin = data["origin"]
        geo.rotation_matrix = data["rotation"]
        geo.valid = True
        geos[bs_id] = geo

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

    # Step 4: Reset Kalman
    reset_kalman(cf)
    time.sleep(1)
    cf.close_link()
    print("Drone rebooted, geometry restored, estimator reset. Ready to fly!")


if __name__ == "__main__":
    main()
