# Disturbance Visualization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Add independent red 3D radial propagation markers to the existing dual-aircraft MuJoCo window.

**Architecture:** A bounded deterministic emitter consumes same-time log snapshots and writes visual geoms only. The CLI updates it under the existing viewer lock; the panel controls its visibility independently of interaction mode. Headless runs skip the emitter.

**Tech Stack:** Existing Python/env_uav, NumPy, MuJoCo 3.6, Matplotlib; no new dependencies.

**Spec:** [Approved design](../specs/2026-11-13-disturbance-visualization-design.md), confirmed by the user on 2026-11-13.

## Global Constraints

- Independent project only; no original-project writes or PX4 recompilation.
- 10 m display radius, 0.6 m launch offset, 2 m/s illustrative propagation.
- World-space emission origins, simulation-time motion, bounded marker count, gradual fade after source stops.
- Default visible; independent of interaction mode; disabling clears visual state immediately.
- Rendering must not advance or modify physics; no new collision bodies.
- Offline/preview verification and actual PX4/desktop acceptance reported separately.

## Review Focus

- Irregular refresh and long time gaps must not cause a burst of fabricated historical emissions.
- Source movement or one source stopping must not move or remove the other source's existing emissions.
- Low geometry capacity must truncate display safely without overwriting the physical scene.
- Toggle off/on and simulation-time reset must not resurrect old markers.
- Grounded or rotor-inactive sources must not emit, even when the visual checkbox is enabled.

### Task 1: Bounded visual emitter

**Files:** Create `src/dual_uav_interaction/disturbance_visualization.py`, `tests/test_disturbance_visualization.py`.

**Interfaces:** `DisturbanceVisualization.update(time_s: float, positions: FloatArray, source_active: Sequence[bool]) -> None`; `draw(scene: mujoco.MjvScene, *, append: bool = False) -> int`; `set_enabled(enabled: bool) -> None`. Draw consumes internally stored world-space emission batches; returns marker geometry count.

- [x] Write failing tests for 3D directions, expanding/fading geometry within 10 m, moving sources, independent source gates, stop-and-expire, toggle/reset, long gaps and bounded scene capacity.
- [x] Run the new tests and confirm the missing-module failure.
- [x] Implement fixed sphere directions, 0.6 s emission intervals, up to 8 batches of 24 directions per aircraft, short capsule trails and radial arrowheads. Expire after (10 - 0.6) / 2 seconds; clip trails so all endpoints remain in radius. No random sampling or model/data references.
- [x] Run focused tests.

### Task 2: Runtime and control integration

**Files:** Modify `simulation.py`, `cli.py`, `panel.py`, `tests/test_panel.py`, `tests/test_disturbance_visualization.py`.

**Interfaces:** Same-time samples add `disturbance_source_active: bool` per aircraft. `ControlPanel(app, visualization=None)` optionally controls a visualization instance; runtime creates one only for graphical runs.

- [x] Add tests showing source gate independent of force mode, panel toggle clears visual state without changing interaction, and two identical physics runs remain equal with/without display updates.
- [x] Add only the source-active diagnostic to samples; reuse the existing gate used by force calculation.
- [x] Create the emitter for graphical sessions, pass it to the panel, update per physical sample, and draw/replace `viewer.user_scn` under its lock before sync. Toggle clears the user scene under the same lock.
- [x] Add a checkbox and explanatory label in free panel space; preserve all existing controls.
- [x] Run focused tests and full existing offline suite.

### Task 3: Preview and documentation

**Files:** Create `scripts/preview_disturbance.py`; update `README.md`, `docs/VALIDATION.md` and the approved spec status; create preview images under `docs/media/`.

- [x] Render actual packaged aircraft with deterministic illustrative airborne poses, offscreen using EGL or Xvfb; do not start PX4. Include close and 10 m overview images plus a short animation if practical.
- [x] Inspect images and adjust density/size/camera only if needed; save a panel preview to check layout.
- [x] Run the offline suite, `--check --mode both`, and a bounded real viewer open/sync/toggle/close smoke test without PX4 if supported.
- [x] Review changes for display-only isolation, scene capacity, reset/teardown, and docs accurately distinguishing visual demonstration from calibrated wind and actual flight acceptance.
- [x] Record exact verification results and remaining desktop/PX4 acceptance. No git commits: this deployment directory is not a Git repository.
