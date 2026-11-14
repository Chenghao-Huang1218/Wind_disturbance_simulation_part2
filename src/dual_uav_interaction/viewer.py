"""Managed MuJoCo 3.6 viewer teardown, copied from the original simulation."""
from __future__ import annotations
import sys
import queue
import threading
from typing import Any
import mujoco
import mujoco.viewer

class ManagedPassiveViewer:
    """Owns the passive render thread until native window teardown completes.

    MuJoCo 3.6's public launch_passive starts an unowned daemon thread, and
    Handle.close only signals exit. Its GLFW atexit callback can consequently
    race render_loop. Keep the installed launcher's setup, but own and join a
    non-daemon thread. The private API is isolated here for version review.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        """Starts the Linux viewer and waits for its handle or startup error.

        Args:
            model: Loaded MuJoCo model to render.
            data: Matching simulation state; no physics is advanced here.

        Raises:
            RuntimeError: If the launcher is unavailable or rendering fails.
        """
        if sys.platform == "darwin":
            raise RuntimeError("ManagedPassiveViewer requires Linux/Windows")
        launcher = getattr(mujoco.viewer, "_launch_internal", None)
        if not callable(launcher):
            raise RuntimeError("MuJoCo viewer launcher is unavailable")
        mujoco.mj_forward(model, data)
        ready: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._thread_error: BaseException | None = None

        def render() -> None:
            """Runs native setup/render/teardown and reports Python failures."""
            try:
                launcher(
                    model, data, run_physics_thread=False, handle_return=ready,
                )
            except BaseException as error:
                self._thread_error = error
                ready.put(error)

        self._thread = threading.Thread(
            target=render, name="px4-mujoco-viewer", daemon=False,
        )
        self._thread.start()
        result = ready.get()
        if isinstance(result, BaseException):
            self._thread.join()
            raise RuntimeError(f"MuJoCo viewer startup failed: {result}") from result
        self._handle = result

    def __getattr__(self, name: str) -> Any:
        """Forwards scene, lock and sync access to the native viewer handle."""
        return getattr(self._handle, name)

    def is_running(self) -> bool:
        """Reports whether the viewer remains open without a render failure."""
        return self._thread_error is None and self._handle.is_running()

    def close(self) -> None:
        """Requests close and waits for native teardown before returning."""
        if self._thread.is_alive():
            self._handle.close()
        self._thread.join()
        if self._thread_error is not None:
            raise RuntimeError(
                f"MuJoCo render thread failed: {self._thread_error}"
            ) from self._thread_error
