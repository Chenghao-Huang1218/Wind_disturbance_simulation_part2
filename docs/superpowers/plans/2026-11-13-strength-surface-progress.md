# Execution ledger — plan: 2026-11-13-strength-surface.md

- User selected the scalar-strength surface + contour option, after reference-image and graph/domain distinction were explained. Execution inline, in the existing non-Git standalone directory.
- Pre-flight: existing class update/draw/toggle API reused; update adds applied L. CLI supplies current source gates and L; panel callback remains unchanged.
- Ruling: replace emission particles with current-position graphs; 10 m is the horizontal scalar graph domain, since graph height is visual encoding rather than a physical airflow coordinate. Documented in design and README; no physical force change.
- Task 1 RED: new tests fail importing missing surface_height before implementation. First surface implementation passed 7 focused tests and full 36-test suite.
- Geometry: 5,088 dual-source geoms at default L=2; cached local polar mesh, vectorized world translation and color preparation. Native user scene capacity is 100,000.
- Native Xvfb/TkAgg check passed: both-source drawing, no arrow geoms, immediate hide/show, L=3 single-source rebuild and normal panel/viewer shutdown. No PX4, bridge connection or mj_step. Cached draw median was 31.3 ms in that local window smoke.
- Independent reviewer ran 10 focused tests and found one P2: analytical contour heights could lie below interpolated triangles at small L. No other material integration/physics issue found.
- Final fixed P2: contour height interpolates the actual mesh radial knots before adding a 0.012 m display lift. Five real-geometry/barycentric regression cases (L=0.1/0.5/1/2/8) reproduced the defect at the first three values before the fix.
- Task 2: actual-model EGL near/overview PNGs and a 24-frame camera-orbit GIF generated; post-contour-fix previews regenerating. Blue checker resource is project-local. README/provenance updated; validation record to follow final verification.
- Final verification: 41 passed in 8.13 s. All five contour cases pass after the fix. Final EGL images/GIF regenerated; final panel preview saved. Offline model check passed (2 aircraft, 8.0004 kg each, dt 0.005 s, no PX4).
- Task 1: complete. Task 2: complete. Documentation distinguishes scalar graph height/horizontal domain from actual physics, and retains prior historical flight evidence without claiming a new flight run. No further blocking findings after the P2 regression fix.
