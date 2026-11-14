# Display Performance Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans inline. Steps use checkbox syntax for tracking.

**Goal:** Reduce GUI cost without changing physics, field geometry or flight limits.
**Architecture:** Pure wall-clock DisplaySchedule shared by CLI/test harness; flat status-region partial redraw; owned native scene geometry cache. Offscreen append path rebuilds safely.
**Tech Stack:** Existing Python/MuJoCo/Matplotlib only.
**Spec:** ../specs/2026-11-13-display-performance-design.md

## Global Constraints

World step remains .005s once per iteration; native PX4/own mass calibration, all forces and loaded .25m/s/.1m/s² remain unchanged. Grid12×48, radius10m, root offset.05m preserved. No original project writes or firmware build. No usable Git; baseline backed up in /tmp/dual-display-perf-baseline.

## Review Focus

- Local redraw must not erase widget edits, button highlights or exported text.
- Resize/savefig/nonblitting backend must remain correct.
- Cache invalidates for different scenes, L, visible source topology and external clear.
- Fade/position updates must match uncached triangle and line transforms and alpha.
- Fast event pumping must retain cancellation, input responsiveness and no window raising.

## Task1: Independent schedule and local text redraw

- [x] RED tests for panel2Hz/events30Hz/viewer20Hz/field10Hz scheduling and skipping missed ticks.
- [x] RED tests for status refresh avoiding full canvas draw and retaining widget pixel regions/savefig.
- [x] Implement DisplaySchedule and panel partial status rendering with safe fallback.

## Task2: Scene reuse and integration

- [x] RED tests for cached no-reinit refresh and numerical equivalence to full build, fade/toggle/L/capacity/append.
- [x] Implement owned-scene transform/alpha cache, leave append path conservative.
- [x] Integrate schedule in CLI and native validation harness, add lightweight wall/sim timing diagnostics to summaries/logs.

## Task3: Evidence and final review

- [x] Full offline suite and identical Xvfb stage timing benchmark before/after.
- [x] Actual native dual PX4 lifecycle including field drawing and six independent moves, report actual sim/wall ratio.
- [x] One fresh-context read-only review; fix material findings with regressions.
- [x] Update documentation and evidence; protected/runtime hashes and owned process cleanup.
