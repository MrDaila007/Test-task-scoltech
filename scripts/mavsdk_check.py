#!/usr/bin/env python3
"""Drive fc_stub with the real MAVSDK, as an onboard computer would.

Starts the stub (real time, default ports as PX4 SITL: listens on 14580, sends to
14540), connects MAVSDK on udp://:14540 and checks: arm, OFFBOARD with velocity
setpoints, the vehicle climbs and moves, HOLD on request, a plain disarm in the
air is denied. Exits 0 when every step passed.

    python3 -m venv .venv && .venv/bin/pip install "mavsdk>=2.8,<3"
    .venv/bin/python scripts/mavsdk_check.py [path/to/fc_stub]
"""

import asyncio
import os
import subprocess
import sys
from typing import AsyncIterator, Callable, TypeVar

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import VelocityNedYaw
from mavsdk.telemetry import FlightMode

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T = TypeVar("T")
STUB = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "build/release/src/fc_stub")


def step(name: str, ok: bool, detail: str = "") -> None:
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}".rstrip())
    if not ok:
        raise SystemExit(1)


async def first(aiter: AsyncIterator[T], predicate: Callable[[T], bool], timeout_s: float) -> T:
    async def scan() -> T:
        async for item in aiter:
            if predicate(item):
                return item
        raise RuntimeError("telemetry stream ended")

    return await asyncio.wait_for(scan(), timeout_s)


async def check() -> None:
    drone = System()
    await drone.connect(system_address="udp://:14540")
    await first(drone.core.connection_state(), lambda s: s.is_connected, 10)
    step("connected", True)

    await asyncio.sleep(2.5)  # the stub becomes ready to arm 2 s after boot
    await drone.action.arm()
    armed = await first(drone.telemetry.armed(), lambda a: a, 5)
    step("arm", armed)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -1.0, 0.0))
    await drone.offboard.start()
    mode = await first(drone.telemetry.flight_mode(), lambda m: m == FlightMode.OFFBOARD, 5)
    step("offboard", mode == FlightMode.OFFBOARD)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(2.0, 0.0, -1.0, 0.0))
    await asyncio.sleep(3.0)
    vel = await first(drone.telemetry.velocity_ned(), lambda _: True, 2)
    step("vehicle follows the setpoint", abs(vel.north_m_s - 2.0) < 0.3 and vel.down_m_s < -0.7,
         f"vn={vel.north_m_s:.2f} vd={vel.down_m_s:.2f} m/s")
    in_air = await first(drone.telemetry.in_air(), lambda _: True, 3)
    step("in_air from EXTENDED_SYS_STATE", in_air)

    await drone.offboard.stop()  # MAVSDK switches to HOLD
    mode = await first(drone.telemetry.flight_mode(), lambda m: m == FlightMode.HOLD, 5)
    step("hold", mode == FlightMode.HOLD)

    try:
        await drone.action.disarm()
        step("disarm in the air is denied", False, "it was accepted")
    except ActionError as e:
        step("disarm in the air is denied", True, str(e).split(":")[0])


def main() -> None:
    stub = subprocess.Popen([STUB, "--config", os.path.join(ROOT, "config/default.yaml")],
                            stderr=subprocess.DEVNULL)
    try:
        asyncio.run(asyncio.wait_for(check(), 60))
        print("all MAVSDK checks passed")
    finally:
        stub.terminate()
        stub.wait(5)


if __name__ == "__main__":
    main()
