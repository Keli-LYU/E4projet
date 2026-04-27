#!/usr/bin/env python3
"""WSL2 helper for connecting ROS launch files to CARLA on Windows."""

from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_PORT = 2000
DEFAULT_TIMEOUT = 0.5


def _read_resolv_conf_nameserver() -> str | None:
    try:
        for line in Path("/etc/resolv.conf").read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if len(parts) >= 2 and parts[0] == "nameserver":
                return parts[1]
    except OSError:
        return None
    return None


def _read_default_gateway() -> str | None:
    try:
        output = subprocess.check_output(
            ["ip", "route", "show", "default"],
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None

    parts = output.split()
    if "via" in parts:
        via_index = parts.index("via")
        if via_index + 1 < len(parts):
            return parts[via_index + 1]
    return None


def candidate_hosts() -> list[str]:
    """Return likely CARLA hosts.

    Explicit environment overrides are tried first. Without an override, prefer a
    Linux/WSL-local CARLA server, then fall back to the Windows host reachable
    through the WSL2 gateway.
    """
    candidates = [
        os.environ.get("CARLA_HOST"),
        os.environ.get("WSL_CARLA_HOST"),
        "127.0.0.1",
        "localhost",
        _read_resolv_conf_nameserver(),
        _read_default_gateway(),
        "host.docker.internal",
    ]

    seen: set[str] = set()
    result: list[str] = []
    for host in candidates:
        if host and host not in seen:
            seen.add(host)
            result.append(host)
    return result


def tcp_open(host: str, port: int = DEFAULT_PORT, timeout: float = DEFAULT_TIMEOUT) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def detect_carla_host(
    port: int = DEFAULT_PORT,
    timeout: float = DEFAULT_TIMEOUT,
    require_open: bool = False,
) -> str:
    """Pick a CARLA host for WSL2.

    When CARLA is already running, the first reachable candidate is returned.
    If nothing is reachable yet, fall back to the WSL2 nameserver/default
    gateway so the launch default still points at the Windows host.
    """
    candidates = candidate_hosts()
    for host in candidates:
        if tcp_open(host, port=port, timeout=timeout):
            return host

    if require_open:
        raise RuntimeError(
            f"CARLA is not reachable on port {port}; tried: {', '.join(candidates)}"
        )

    return _read_resolv_conf_nameserver() or _read_default_gateway() or "127.0.0.1"


def sanitize_pythonpath(value: str | None = None) -> str:
    """Remove CARLA wheel zip paths that shadow the installed carla package."""
    if value is None:
        value = os.environ.get("PYTHONPATH", "")
    keep = [
        entry
        for entry in value.split(os.pathsep)
        if entry and not (entry.endswith(".whl") and "/PythonAPI/carla/dist/" in entry)
    ]
    return os.pathsep.join(keep)


def apply_pythonpath_sanitization() -> None:
    os.environ["PYTHONPATH"] = sanitize_pythonpath()
    sys.path[:] = [
        entry
        for entry in sys.path
        if not (entry.endswith(".whl") and "/PythonAPI/carla/dist/" in entry)
    ]


def wait_for_carla(host: str, port: int, deadline_seconds: float) -> bool:
    deadline = time.monotonic() + deadline_seconds
    while time.monotonic() <= deadline:
        if tcp_open(host, port=port, timeout=min(DEFAULT_TIMEOUT, deadline_seconds)):
            return True
        time.sleep(0.5)
    return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--require-open", action="store_true")
    parser.add_argument("--wait", type=float, default=0.0, metavar="SECONDS")
    parser.add_argument("--print-host", action="store_true")
    parser.add_argument("--export", action="store_true", help="print shell exports")
    parser.add_argument("--check-python", action="store_true", help="check CARLA Python API import")
    args = parser.parse_args(argv)

    try:
        host = detect_carla_host(
            port=args.port,
            timeout=args.timeout,
            require_open=args.require_open and args.wait <= 0,
        )
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 2

    if args.wait and not wait_for_carla(host, args.port, args.wait):
        print(f"Timed out waiting for CARLA at {host}:{args.port}", file=sys.stderr)
        return 2

    if args.check_python:
        apply_pythonpath_sanitization()
        try:
            import carla  # noqa: F401
        except Exception as exc:  # pragma: no cover - diagnostic path
            print(f"CARLA Python API import failed: {exc}", file=sys.stderr)
            return 3

    if args.export:
        print(f"export CARLA_HOST={host}")
        print(f"export CARLA_PORT={args.port}")
        print(f"export PYTHONPATH={sanitize_pythonpath()!r}")
    else:
        print(host if args.print_host else f"CARLA host: {host}:{args.port}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
