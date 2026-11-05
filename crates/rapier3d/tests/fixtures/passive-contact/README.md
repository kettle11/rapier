# Passive joint/contact regression

## Archived experiment — 2026-11-05

The Offroad owner shelved this fork for initial launch and retained stock Rapier
0.35.3 for simplicity and safety. The implementation and regression tests remain
available for post-launch research, not as an approved application dependency.
The original implementation is `a26449b51e87b3cda844550c6b5e170e3ed6a9e1`;
`archive/offroad-prelaunch-2026-11-05` preserves it with this documentation notice.

Later application validation found sustained powered turns about 23% slower and
slalom speed about 15% lower than both stock 0.35.3 and untouched upstream 0.36.1.
Ablations isolated that difference to freezing friction warm-start estimates;
it is not established as a more physically accurate tire response. Normal-only
protection restored turn speed and passed the five crash cases, but introduced
a thin-slab tunneling regression. A contact-history draft passed those collision
cases but retained a substantial turn-speed change. Neither draft was adopted.

Two additional upstream issues were investigated separately: incomplete support
features for many-vertex convex wheel caps, and extreme-substep joint recovery
velocities interacting with the linear speed cap. They do not justify weakening
collision assertions or disabling the cap. The earlier validation below records
what this patch passed, not a complete application adoption gate.

Reconsider only after preserving ordinary handling while passing the original
energy cases, thin-slab/core collision suites, native/Wasm parity and application
integration checks, with fresh performance measurements. Detailed application
records and unvalidated patches are retained in the local Offroad archive:
`/Users/ian/Documents/Workspace/lolo/output/rapier-archive-2026-11-05/`.

## Original investigation and validation

A light wheel hitting a surface while attached to a much heavier chassis could
create enough kinetic energy to launch the chassis. A two-body passive reproduction
added 155 kJ in one 1/120-second step, with zero restitution and no motors or springs.
Starting before contact, with less than 0.05 mm of locked-axis position error,
also reproduced the energy gain.

Contact warmstarting reused each substep's solved impulse as the next substep's
initial guess. A transient impact or joint stabilization impulse could therefore
feed back through successive substeps. The finite joint/contact solve did not
remove it reliably at this mass ratio. Angular locks also had unbounded positional
recovery, even though angular limits already bounded recovery.

The fix keeps normal, tangent and twist warmstart estimates fixed from the previous
completed solver step. Each substep's solved impulses are still banked separately
for contact-force reporting. The new estimates become available at the next step.
Angular-lock recovery follows the existing angular-limit policy, with its bound
multiplied by the angular Jacobian norm: a quaternion-component derivative is not
a unit angular-velocity row. The bound applies to the positional correction term; physical velocities and
the iteration budget retain their existing rules. Baumgarte stabilization
still has no global energy-conservation guarantee; the tests cover this impact
and its variants.

The fixtures contain two dynamic bodies (approximately 978 kg and 18 kg), one
passive slider/axle joint, a rounded convex hull and a fixed half-space. The hull
has two 128-vertex rings; coordinates are retained verbatim to avoid dependence
on trigonometric reconstruction. There are no application dependencies, tire
hooks or serialized broad-phase/contact/impulse caches.

`scene.json` starts at impact. `approach-scene.json` starts two world steps earlier;
three steps reach the collision. `wheel-points.json` is shared by both. The tests
reconstruct all geometry and bodies using public Rapier APIs, verify that contact
actually occurs and that states remain finite, and measure kinetic plus gravitational
potential energy. They use dt=1/120 s, four internal PGS passes, and four additional
solver iterations on the heavy body. Other solver parameters retain their defaults.

| Passive case | Upstream energy change | Fork energy change |
|---|---:|---:|
| Impact | +155.5 kJ | −35.7 kJ |
| Approach, three steps | +22.9 kJ | −34.3 kJ |
| Impact with initial angular velocities zeroed | +170.6 kJ | −5.9 kJ |
| Frictionless impact | +158.6 kJ | −34.1 kJ |
| Coulomb-friction impact | +184.3 kJ | −35.6 kJ |

The energy assertions permit 1 J of numerical noise relative to roughly 100 kJ
incident kinetic energy. A 16-step check covers delayed launches in both friction
models, without friction, and without initial spin. Unit tests additionally check
nonzero persistent warmstarts, coefficients 0/0.5/1, impulse accumulation, angular
recovery units, preservation of small corrections, and removal of bias in relaxation.
All eight added regression tests fail with the original solver behavior.

```sh
cargo test --release -p rapier3d --features serde-serialize,enhanced-determinism --test passive_joint_contact -- --nocapture
cargo test --release -p rapier3d --features serde-serialize,enhanced-determinism --lib --tests
cargo test --release -p rapier3d --features serde-serialize --lib --tests
```

Validation on 2026-11-05 used rustc 1.97.0-nightly (7af3402cd, 2026-04-16),
Rapier base commit `475caa3ff4646fd9a796b67d14116256f4962bc7`, and Parry 0.31.1.
The enhanced-determinism 3D core suite passed 375 tests (8 existing manual/timing
tests ignored). The ordinary 3D feature configuration passed 372 tests. Both f64
libraries passed their unit suites. Parallel execution and thread-count parity
passed. The intentional simulation/snapshot hash updates agree on ARM, x86 and
wasm32-wasip1; the 2D snapshot also agrees between ARM and Wasm. The browser
wasm32-unknown-unknown target builds successfully.

```sh
cargo test --release -p rapier2d-f64 -p rapier3d-f64 --features serde-serialize,enhanced-determinism --lib
cargo test --release -p rapier3d --features serde-serialize,enhanced-determinism,parallel --lib --test passive_joint_contact --test parallel_path_parity --test thread_count_determinism --test simd_backend_determinism --test snapshot_portability --test joint_contact_solve_order --test joint_stability
cargo test --release -p rapier3d --features serde-serialize,enhanced-determinism --target x86_64-apple-darwin --test passive_joint_contact --test parallel_path_parity --test simd_backend_determinism --test snapshot_portability --test simd_backend_parity
# Set CARGO_TARGET_WASM32_WASIP1_RUNNER to a WASI runtime.
cargo test --release -p rapier3d --features serde-serialize,enhanced-determinism --target wasm32-wasip1 --lib --test passive_joint_contact --test parallel_path_parity --test simd_backend_determinism --test snapshot_portability -- --test-threads=1
cargo test --release -p rapier2d --features serde-serialize,enhanced-determinism --target wasm32-wasip1 --lib --test snapshot_portability -- --test-threads=1
cargo check --release -p rapier3d --features serde-serialize,enhanced-determinism --target wasm32-unknown-unknown
```

The full 2D suite has one pre-existing failure:
`soft_bodies::neo_hookean_square_survives_large_compression`. Its compression-height
check reports `0.55116487` instead of `<0.45`, identically in this fork and a freshly
built untouched upstream checkout with a separate target directory. The existing
assertion remains unchanged. The 2D snapshot expectation changed with the physics and was
updated only after native/Wasm verification.

A separate application snapshot (505 bodies, 535 colliders, 23 joints) was checked
by backporting the same behavioral changes to an isolated copy of 0.35.3, whose
binary snapshot format differs from 0.36.1. At the captured collision, the energy
change went from +179.9 kJ to −39.9 kJ; chassis vertical velocity went from
+14.27 m/s to −1.86 m/s. The application dependency was not changed.

An interleaved native benchmark of the existing determinism scene (432 boxes and
a joint chain, 120 steps and scene creation per sample, eight alternating pairs of ten-sample batches)
measured medians of 13.77 ms upstream and 13.29 ms in the fork. No slowdown was
observed in this workload. Simulation trajectories differ, and the measurement
does not establish application frame-time performance.
