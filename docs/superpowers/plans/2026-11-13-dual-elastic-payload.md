# Dual Elastic Payload Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Two same-heading side-by-side X4-950/Airbot aircraft lift, translate and land a shared block on two gripper-attached elastic ropes.

**Architecture:** A pure unilateral spring/damper law and a MuJoCo payload binding layer apply equal/opposite point forces through a rope-owned generalized-force accumulator. The existing shared loop and two independent PX4 bridges remain; a testable transport sequencer verifies cargo lift, actual common translation and settled landing.

**Tech Stack:** Existing Python/env_uav, NumPy, MuJoCo 3.6, Matplotlib; copied PX4 runtime unchanged.

**Spec:** [Approved design](../specs/2026-11-13-dual-elastic-payload-design.md), user confirmed implementation and complete dual-PX4 flight validation.

## Global Constraints

- Work only in this standalone project; reference projects read-only, no PX4 recompilation.
- World Y separation, heading +X/yaw0; initial separation2m. Loaded initial/target separation1.5..2.25m; unloaded target range unchanged.
- Payload .5kg, .40x.30x.15m, top sites y=+/-.12m; gripper endpoints a/endpoint,b/endpoint.
- Each rope L0=1.2m, k=100N/m, c=3N*s/m, tension-only; keep gravity9.80665 and shared dt.005s.
- qfrc_applied owned by ropes: clear before leading mj_forward, sum both ropes, write once; preserve aerodynamic xfrc channel.
- Apply at actual link6/payload sites; never move reaction to root or zero rope-induced COM moments. Subtree thrust calibration excludes cargo.
- Native tendons draw only (stiffness=damping=0). One mj_step per shared step. No L1, training, real hardware, physical grasp/release or new payload controller.
- Default CLI loads cargo; --no-payload gives unloaded comparison. Diagnostics/metadata distinguish cargo and aero loads.

## Review Focus

- Slack/contracting ropes and time-step force clearing must never push or retain stale force.
- Off-axis attachment motion must include angular/joint velocity and correct generalized point-force mapping.
- Initialization/runtime spacing validation must agree; custom params cannot create initial prestretch/penetration silently.
- Automated success must require actual cargo lift, displacement and settled floor contact; no false success from aircraft hover/landed alone.
- Source failures, signals and new validation timeouts must clean only owned PX4 processes and retain honest failure records.

### Task1: Rope law, payload scene and bindings

**Files:** Create `src/dual_uav_interaction/payload.py`, `tests/test_payload.py`; modify `scene.py`, `tests/test_scene.py`.
**Interfaces:** `ElasticRopeConfig`, `PayloadConfig`; `compute_rope_force(anchor_position, payload_position, anchor_velocity, payload_velocity, config) -> RopeState`; `PayloadSystem(model, config)` with `apply(data) -> (payload_sample, rope_samples)`; `load_scene(separation_m=2, payload_config=None)` preserves existing three-item return.

- [x] Write/run failing pure rope and real-model tests for slack, extension/damping, zero length, invalid inputs, equal/opposite forces, Y/same-heading layout, default slack/contact geometry and inert tendons.
- [x] Implement finite config/state validation, native cargo sites/freejoint/tendons and actual point velocity/applyFT; test Jacobian mapping and upstream arm loads.
- [x] Run focused tests and a fixed-anchor physical equilibrium check; retain first-principles force/moment and no-double-force checks.

### Task2: Shared simulation and complete transport sequence

**Files:** Modify `simulation.py`, `cli.py`, `panel.py`, existing tests; create `transport.py`, `tests/test_transport.py`.
**Interfaces:** `DualSimulation(..., payload_config=None)` keeps library tests unloaded unless explicitly requested; CLI supplies config by default. Samples add `payload` (or None) and `ropes`, each aircraft adds its rope force/COM moment. `TransportSequence` consumes snapshots and app commands, with explicit sustained lift/transfer/floor-settling stages.

- [x] Test force clearing/addition versus aero isolation, single shared step, loaded spacing constraints and configured no-payload path; update layout-dependent expectations to Y.
- [x] Add rope application after current snapshot/control/aero calculation and before one mj_step. Expose config CLI options and cargo status/readouts.
- [x] Implement/test auto stage sequence: lift sustained1s -> common X+.5m -> targets and cargo settle1s with >=.35m cargo displacement -> requested hold -> land -> payload contact/low-speed sustained.7s plus both aircraft landed.
- [x] Metadata and summary record all success predicates and measured cargo displacement/clearance/tension, retaining unloaded workflow.
- [x] Run full offline suite and --check for loaded/unloaded models.

### Task3: Actual validation, previews and documentation

**Files:** Update README/PROVENANCE/VALIDATION/spec status; create records under `outputs/validation` and cargo images under `docs/media`.

- [x] Run bounded owned dual PX4 off-mode carrying sequence (instances4/5), inspect state logs for true lift/translation/contact, resolve any failure systematically.
- [x] Request one independent code review; fix material findings with focused RED-GREEN regression tests and full suite.
- [x] Run final loaded both-mode sequence and an unloaded regression as justified; obtain graphical carried-model preview from actual run states, verify UI layout and rope endpoints.
- [x] Verify original protected and PX4 runtime hashes, owned-process cleanup, test/flight records and documented limits. No physical success claims from illustrative poses.

Execution inline in the existing standalone directory, which has no Git repository. Progress/decisions stored in a sibling progress ledger; no worktree or commit operations.
