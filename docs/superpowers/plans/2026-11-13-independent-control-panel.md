# Independent Dual Control Panel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline. Steps use checkbox syntax for tracking.

**Goal:** Reference-style two-aircraft XYZ panel, one-click paired takeoff and explicit field start/stop.
**Architecture:** Existing Matplotlib panel controls shared simulation through validated per-aircraft commands. A nonblocking paired takeoff request waits for real arm confirmation. Display callback stays separate from physics.
**Tech Stack:** Python, NumPy, Matplotlib, existing MuJoCo/PX4.
**Spec:** ../specs/2026-11-13-independent-control-panel-design.md

## Global Constraints

Original projects read-only; shared mj_step once; identities/actuators isolated. Keep payload/interaction formulas and native PX4 unchanged. World XYZ, loaded target spacing1.5..2.25, unloaded1.5..10. No firmware builds. Repository has no usable Git metadata; edit standalone workspace, retain backups.

## Review Focus

- A button must never alter B target, including rejected moves.
- Late arm ACK must not take off after Land/Emergency/Exit.
- Triangle whitespace/right clicks must not move the aircraft.
- Start/stop field must match viewer visibility and leave physics settings untouched.
- Draft settings and status text must remain usable without overlap at native font metrics.

## Task 1: Command interface

- [x] RED tests for isolated XYZ motion, spacing rejection, finite target arithmetic, paired takeoff acknowledgement/cancellation.
- [x] Add move_aircraft(index, offset) and request_takeoff command; process pending request after both bridge updates, before source/force snapshot. Retain existing arm/takeoff and automated lifecycle.
- [x] Focused tests GREEN.

## Task 2: Reference-style panel

- [x] RED mouse-event tests for both triangular direction pads, selectable steps, explicit field toggle and error handling.
- [x] Rewrite panel with two pads, aircraft position/target status, common actions, lower settings/cargo diagnostics. Field initially off in GUI launcher.
- [x] Render and inspect layout, exercise hover/click callbacks and existing atomic settings behavior.

## Task 3: Validation and delivery

- [x] Full offline suite, native viewer/panel automated smoke, targeted actual two-PX4 independent motion lifecycle if supported by bounded harness.
- [x] One independent final read-only code review; fix material issues with regressions.
- [x] README/PROVENANCE/VALIDATION and preview/evidence updates, protected/runtime hashes and owned-process cleanup.
