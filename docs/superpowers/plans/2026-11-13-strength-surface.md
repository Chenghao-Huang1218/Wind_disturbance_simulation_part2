# Strength Surface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Replace arrow particles with translucent scalar-strength surfaces and contours on the restored blue checker scene.

**Architecture:** Reuse the display-only class API; cache a local polar mesh from the applied L and translate it per source. Render bounded right-triangle geoms and contour lines into user_scn; state uses current position plus source-stop fading.

**Tech Stack:** Existing MuJoCo 3.6, NumPy, Matplotlib/Pillow; no new dependencies.

**Spec:** [Selected design](../specs/2026-11-13-strength-surface-design.md).

## Global Constraints

- No physics/force-model change, original-project writes or PX4 recompilation.
- Horizontal graph radius 10 m; z=-0.65-1.35*(1-g(r)) relative to source. This is a scalar height graph, not flow geometry.
- Independent per-source gate; interaction mode does not control visibility. Applied L controls graph width.
- No arrows; translucent warm surfaces and contour rings; clear on toggle; current positions, bounded stop fade and capacity.
- Keep default graphical startup and headless behavior, one shared physics step.

## Review Focus

- Triangle primitive requires orthogonal legs; arbitrary triangles must be split without holes or degenerate normals.
- Transparent overlapping surfaces must not obscure aircraft or flicker from contour z-fighting.
- Scene capacity must preserve appended physical geometry and retain bounded work.
- L edits, source movement/stopping, visibility resets and time rewind must not resurrect stale surface data.
- Metadata and docs must describe horizontal graph radius and scalar encoding, excluding physical potential/wind-speed claims.

### Task 1: Surface geometry and display state

**Files:** Replace `src/dual_uav_interaction/disturbance_visualization.py`, update `tests/test_disturbance_visualization.py`.
**Interfaces:** `surface_height(radius_m: FloatArray, length_m: float) -> FloatArray`; existing `update(time_s, positions, source_active, length_m=2.0)`, `set_enabled`, `draw(scene, append=False)`.

- [x] Update behavioral tests to assert literal height values at r=0/2/6 for L=2, monotonicity and L half-amplitude; no arrow geoms; translucent triangles and contours; reconstructed triangle bounds and orthonormal frames; independent source fades, live positions and resets; append capacity and unchanged physics.
- [x] Run focused tests to observe expected missing-function/new-behavior failures.
- [x] Implement cached polar mesh (12 radial intervals, 48 azimuth sectors), robust longest-edge right-triangle subdivision, thin contour rings, per-source warm palettes and 0.6 s stop fading. Use explicit checks for invalid numeric inputs; no randomness.
- [x] Run focused tests, then integrate applied L and visual metadata.

### Task 2: Runtime/panel and previews

**Files:** Modify `cli.py`, `panel.py`, `scripts/preview_disturbance.py`, and documentation/verification records.

- [x] Pass applied L through the graphical update path; update checkbox/status text and metadata to strength-surface semantics.
- [x] Reuse panel behavior/layout regressions; render actual-model PNGs and short preview animation, inspect and adjust purely visual defaults if needed.
- [x] Run full offline suite and model check; run bounded native viewer/panel open/sync/toggle/close smoke without PX4; benchmark display geometry generation.
- [x] Request one read-only final code review, resolve any material findings with focused regression evidence.
- [x] Update README/VALIDATION/provenance and verification JSON, preserving previous historical evidence and clearly reporting no new flight acceptance.

Execution is inline in the existing standalone directory (not a Git repository). No worktree/commit steps apply.
