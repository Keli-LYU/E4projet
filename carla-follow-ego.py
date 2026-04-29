#!/usr/bin/env python3
"""Keep the CARLA spectator camera following the ego vehicle."""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys
import time
from pathlib import Path


def _load_wsl_bridge():
    candidate = Path(__file__).resolve().parent / "projection" / "wsl-bridge" / "wsl-bridge.py"
    if not candidate.exists():
        return None
    spec = importlib.util.spec_from_file_location("projection_wsl_bridge", candidate)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _prepare_carla_import() -> None:
    helper = _load_wsl_bridge()
    if helper is not None:
        helper.apply_pythonpath_sanitization()
    else:
        os.environ["PYTHONPATH"] = os.pathsep.join(
            entry
            for entry in os.environ.get("PYTHONPATH", "").split(os.pathsep)
            if entry and not entry.endswith(".whl")
        )
        sys.path[:] = [entry for entry in sys.path if not entry.endswith(".whl")]


def _detect_host(port: int, timeout: float) -> str:
    helper = _load_wsl_bridge()
    if helper is None:
        return os.environ.get("CARLA_HOST", "127.0.0.1")
    return helper.detect_carla_host(port=port, timeout=timeout)


def _find_ego_vehicle(world, role_name: str):
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


def _spectator_transform(carla, vehicle, distance: float, height: float, pitch: float):
    transform = vehicle.get_transform()
    forward = transform.get_forward_vector()

    location = carla.Location(
        x=transform.location.x - forward.x * distance,
        y=transform.location.y - forward.y * distance,
        z=transform.location.z + height,
    )
    rotation = carla.Rotation(
        pitch=pitch,
        yaw=transform.rotation.yaw,
        roll=0.0,
    )
    return carla.Transform(location, rotation)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000")))
    parser.add_argument("--role-name", default="ego_vehicle")
    parser.add_argument("--distance", type=float, default=12.0)
    parser.add_argument("--height", type=float, default=6.0)
    parser.add_argument("--pitch", type=float, default=-25.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    _prepare_carla_import()
    import carla

    host = args.host or _detect_host(args.port, timeout=0.2)
    client = carla.Client(host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    spectator = world.get_spectator()

    print(f"Following role_name={args.role_name!r} at {host}:{args.port}")
    sleep_seconds = 1.0 / max(args.rate, 1.0)

    try:
        while True:
            ego = _find_ego_vehicle(world, args.role_name)
            if ego is None:
                print(f"Waiting for CARLA actor role_name={args.role_name!r}...")
                time.sleep(1.0)
                continue

            if math.isfinite(args.distance) and math.isfinite(args.height):
                spectator.set_transform(
                    _spectator_transform(
                        carla,
                        ego,
                        distance=args.distance,
                        height=args.height,
                        pitch=args.pitch,
                    )
                )
            time.sleep(sleep_seconds)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
