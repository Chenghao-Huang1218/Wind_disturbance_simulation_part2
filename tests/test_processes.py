"""Owned-process cleanup and independent per-instance state."""

from pathlib import Path
from typing import Any

import pytest

from dual_uav_interaction.processes import SitlPair


class Child:
    pid = 12345

    def __init__(self) -> None:
        self.returncode = None
        self.stopped = False

    def poll(self) -> int | None:
        return self.returncode

    def send_signal(self, value: int) -> None:
        self.stopped = True
        self.returncode = 0

    def wait(self, timeout: float) -> int:
        return int(self.returncode or 0)


def fake_runtime(tmp_path: Path) -> Path:
    runtime = tmp_path / "runtime"
    (runtime / "bin").mkdir(parents=True)
    (runtime / "etc/init.d-posix").mkdir(parents=True)
    (runtime / "bin/px4").touch()
    (runtime / "etc/init.d-posix/rcS").touch()
    return runtime


def test_pair_cleanup_and_separate_state(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        "dual_uav_interaction.processes.check_ports", lambda _: None
    )
    records = []
    children = []

    def spawn(args: list[str], **kwargs: Any) -> Child:
        records.append((args, kwargs))
        child = Child()
        children.append(child)
        return child

    with SitlPair(
        fake_runtime(tmp_path), tmp_path / "state", 0, process_factory=spawn
    ):
        assert records[0][0][-2:] == ["-i", "0"]
        assert records[1][0][-2:] == ["-i", "1"]
        assert all("-d" in record[0] for record in records)
        for args, options in records:
            test_data = options["cwd"] / "test_data"
            assert test_data.is_dir() and not test_data.is_symlink()
            assert args[args.index("-t") + 1] == str(test_data)
        assert records[0][1]["cwd"] != records[1][1]["cwd"]
        assert all(item[1]["start_new_session"] for item in records)
    assert all(child.stopped for child in children)


def test_second_launch_failure_cleans_first(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(
        "dual_uav_interaction.processes.check_ports", lambda _: None
    )
    first = Child()
    count = 0

    def spawn(*args: Any, **kwargs: Any) -> Child:
        nonlocal count
        count += 1
        if count == 2:
            raise OSError("second launch failed")
        return first

    with pytest.raises(OSError):
        with SitlPair(
            fake_runtime(tmp_path),
            tmp_path / "state",
            0,
            process_factory=spawn,
        ):
            pass
    assert first.stopped
