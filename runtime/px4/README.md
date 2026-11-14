# Included PX4 SITL runtime

`bin/` and `etc/` contain the existing Linux x86-64 SITL runtime rebuilt in the
source project on 2026-11-10. This dual-UAV project copies that runtime without
recompiling it and launches the same binary twice with isolated instance IDs,
ports, parameters, and logs. The full PX4 source tree and build cache are not
included.

From the repository root, use:

```bash
python scripts/run_simulation.py --mode both --instance-base 4
```

Do not start separate PX4 processes for the same instances. The launcher owns
and cleans up its two children. Check shared libraries before running:

```bash
ldd runtime/px4/bin/px4
```

Gazebo Transport 13, Messages 10, Utils 2, Protobuf and C/C++ system libraries
must be provided by the host; see `docs/INSTALLATION.md`. Physics runs in
MuJoCo. The runtime uses the existing simulation-truth feedback adaptation,
not the real-sensor EKF2 chain.

Binary SHA256:
`db1c803bb28077856f8989b059c12421e0c8d3815be69aa3bb81b3262ae735ca`.

PX4 is licensed under BSD-3-Clause; see `licenses/PX4-BSD-3-Clause.txt`.
Source baseline, patch, runtime hashes and submodule records are retained in
`docs/provenance/`; project integration details are in `docs/PROVENANCE.md`.
Historical rebuild records describe the source project's build, not a new
build performed in this dual-UAV project.
