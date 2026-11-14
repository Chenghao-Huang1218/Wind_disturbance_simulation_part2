"""Own exactly two PX4 child processes, with independent state and logs."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
from typing import Any, Callable, Self


def check_ports(instance_base: int) -> None:
    """Fail on occupied required localhost endpoints without stopping their owner.

    Args:
        instance_base: First of two consecutive PX4 instance indices.
    """
    for instance in (instance_base, instance_base + 1):
        for kind, port in (
            (socket.SOCK_STREAM, 4560 + instance),
            (socket.SOCK_DGRAM, 14580 + instance),
            (socket.SOCK_DGRAM, 18570 + instance),
            (socket.SOCK_DGRAM, 14280 + instance),
            (socket.SOCK_DGRAM, 13030 + instance),
        ):
            with socket.socket(socket.AF_INET, kind) as candidate:
                if kind == socket.SOCK_STREAM:
                    # Match PX4's listener and permit closed TCP TIME_WAIT,
                    # while still refusing an active listening socket.
                    candidate.setsockopt(
                        socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
                    )
                try:
                    candidate.bind(("127.0.0.1", port))
                except OSError as error:
                    raise RuntimeError(
                        f"PX4 endpoint unavailable on port {port}; existing processes are left untouched"
                    ) from error


def stop_owned_child(child: subprocess.Popen[Any]) -> None:
    """Stop only a process object created by this launcher, using bounded waits."""
    if child.poll() is not None:
        return
    child.send_signal(signal.SIGINT)
    try:
        child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=5)


class SitlPair:
    """Context manager for an isolated pair of locally owned PX4 instances.

    Args:
        runtime: Copied PX4 bin/etc directory.
        state: Fresh run-specific writable directory.
        instance_base: First of two consecutive instance indices, 0..8.
        process_factory: Injectable subprocess constructor for lifecycle tests.
    """

    def __init__(
        self,
        runtime: Path,
        state: Path,
        instance_base: int = 0,
        process_factory: Callable[..., Any] = subprocess.Popen,
    ) -> None:
        if not 0 <= instance_base <= 8:
            raise ValueError("instance_base must be in 0..8")
        self.runtime = runtime.resolve()
        self.state = state.resolve()
        self.instance_base = instance_base
        self.factory = process_factory
        self.children: list[subprocess.Popen[Any]] = []
        self.logs: list[Any] = []

    def __enter__(self) -> Self:
        """Preflight all endpoints and start both children, cleaning partial failure."""
        binary = self.runtime / "bin/px4"
        if (
            not binary.is_file()
            or not (self.runtime / "etc/init.d-posix/rcS").is_file()
        ):
            raise FileNotFoundError(f"incomplete PX4 runtime: {self.runtime}")
        check_ports(self.instance_base)
        self.state.mkdir(parents=True, exist_ok=False)
        try:
            for label, instance in zip(
                ("a", "b"), (self.instance_base, self.instance_base + 1)
            ):
                directory = self.state / label
                directory.mkdir()
                # Override the binary's compiled-in PX4_SOURCE_DIR/test_data.
                # Runtime SITL does not need upstream unit-test fixtures.
                test_data = directory / "test_data"
                test_data.mkdir()
                log = (directory / "console.log").open("w")
                self.logs.append(log)
                environment = os.environ.copy()
                environment.update(
                    PX4_SIM_MODEL="none_iris", PX4_SIM_FLOAT_STATE="1"
                )
                environment["PATH"] = (
                    str(binary.parent)
                    + os.pathsep
                    + environment.get("PATH", "")
                )
                child = self.factory(
                    [
                        str(binary),
                        str(self.runtime / "etc"),
                        "-d",
                        "-s",
                        "etc/init.d-posix/rcS",
                        "-t",
                        str(test_data),
                        "-i",
                        str(instance),
                    ],
                    cwd=directory,
                    env=environment,
                    stdin=subprocess.DEVNULL,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                self.children.append(child)
                print(
                    f"PX4 {label.upper()}: instance={instance}, system={instance+1}, PID={child.pid}",
                    flush=True,
                )
            (self.state / "session.json").write_text(
                json.dumps(
                    {
                        "pids": [child.pid for child in self.children],
                        "instance_base": self.instance_base,
                        "ownership": "informational PIDs only; shutdown uses owned Popen objects",
                    },
                    indent=2,
                )
            )
        except BaseException:
            self.stop()
            raise
        return self

    def stop(self) -> None:
        """Reap both owned children and close their log streams."""
        for child in reversed(self.children):
            stop_owned_child(child)
        for stream in self.logs:
            stream.close()

    def __exit__(self, *exception: Any) -> None:
        """Always stop owned children, including after application failure."""
        self.stop()
        print("Both owned PX4 instances stopped.", flush=True)
