#!/usr/bin/env python3
"""Shared utilities for Crazyflie tool scripts.

Provides:
  - URI argument parsing (--drone N or --uri full_uri)
  - Connection helper with timeout and TOC wait
"""

import argparse
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie


DEFAULT_DRONE = 2
URI_TEMPLATE = "radio://0/80/2M/E7E7E7E7D{}"


def parse_uri_args(description: str) -> str:
    """Parse --drone / --uri from command line and return a URI string."""
    parser = argparse.ArgumentParser(description=description)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--drone", type=int, default=DEFAULT_DRONE,
        help=f"Drone number (0-9). Builds URI radio://0/80/2M/E7E7E7E7D<N>. "
             f"Default: {DEFAULT_DRONE}")
    group.add_argument(
        "--uri", type=str, default=None,
        help="Full Crazyflie URI (overrides --drone)")
    args = parser.parse_args()
    uri = args.uri if args.uri else URI_TEMPLATE.format(args.drone)
    return uri


def connect_and_wait(uri: str, timeout_connect: float = 5.0,
                     timeout_toc: float = 20.0) -> Crazyflie | None:
    """Connect to a Crazyflie and wait for full TOC download.

    Returns the connected Crazyflie object, or None on failure.
    """
    cflib.crtp.init_drivers()
    cf = Crazyflie(rw_cache="./cache")

    connected = False
    fully_connected = False
    failed_msg = None

    def on_connected(link_uri):
        nonlocal connected
        connected = True
        print(f"Connected to {link_uri}")

    def on_fully_connected(link_uri):
        nonlocal fully_connected
        fully_connected = True

    def on_failed(link_uri, msg):
        nonlocal failed_msg
        failed_msg = msg
        print(f"Connection failed: {msg}")

    cf.connected.add_callback(on_connected)
    cf.fully_connected.add_callback(on_fully_connected)
    cf.connection_failed.add_callback(on_failed)

    print(f"Connecting to {uri}...")
    cf.open_link(uri)

    deadline = time.monotonic() + timeout_connect
    while not connected and failed_msg is None and time.monotonic() < deadline:
        time.sleep(0.1)

    if not connected:
        print("Failed to connect!")
        return None

    print("Waiting for TOC...")
    deadline = time.monotonic() + timeout_toc
    while not fully_connected and time.monotonic() < deadline:
        time.sleep(0.1)

    if not fully_connected:
        print("TOC download timed out!")
        cf.close_link()
        return None

    time.sleep(0.5)
    return cf


def reset_kalman(cf: Crazyflie):
    """Reset the Kalman state estimator."""
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.5)
    cf.param.set_value("kalman.resetEstimation", "0")
    print("Kalman estimator reset.")
