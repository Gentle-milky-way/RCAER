# Implementation notes and exactness boundary

This is a clean-room reference implementation of the **paper's method description**; it is **not** the original experiment source, not a validated FR3 controller, and not a full paper reproduction.

| Component | Paper | This repository | Status |
|---|---|---|---|
| Execution interface | Section III-A, Eq. (1) | `engine.decide` accepts a selected grasp and calibrated observation | Implemented core interface |
| Guarded anchoring | Section III-B, Fig. 2, Table III | Local masked 3-D projection; distance-transform interior preference; bounds | Conservative approximation; original selection heuristic not specified |
| Condition descriptor | Section III-C, Eq. (2), Table I | Seven normalized cues from supplied calibrated measurements and local depth | Implemented formulas; some raw sensing estimators are external |
| Risk and routing | Section III-C-D, Eqs. (3)-(4) | `max(z)`, thresholds 0.50 / 0.70 | Exact policy logic |
| Profiles | Table II | All three gains, offsets, probe values, impedance/damping, grip settings | Exact codebook values |
| Bounded projection | Section III-E, Table III | Componentwise clipping on continuous profile settings | Exact manuscript bounds |
| Hard gate | Eq. (5), Table III | Workspace, table, tilt, reachability and planning veto | Core checks; extra robot-specific safety remains external |
| Waypoints | Section III-E | Dry-run phases and Cartesian position intents | NOT controller trajectories |
| Audit | Section IV-H, Table XI | JSON/JSONL decisions; optional external outcome and timing | Implemented schema |
| Replay | Section IV-H, Table X | Leave-one-cue-out and material scenarios | Tier/profile predictions ONLY |

## Measurements not recoverable from the paper alone

- Precise semantic front-end, M2T2 / GraspMolmo weights, target mask selection, camera calibration, and runtime communication formats.
- Binary segmentation thresholds, exact table-depth consistency threshold, mask/depth-based bilateral finger contact area estimator, depth-to-base coordinate registration.
- The four *normalized* robot safety-margin equations inside `zA`. Supply normalized margins in `[0,1]` after *independent* hard-constraint tests.
- Original orientation conventions and hand-eye mapping, IK solutions, collision-checked intermediate paths, recovery controller, live protective stops, Cartesian impedance code, and the exact probing trajectory. The 6-DoF quaternion and calibrated approach-axis direction are carried through but not turned into executable actuator commands.
- Live plant material database integration; this uses an explicit `Material` structure.
- Original trial ledger, synchronized sensor streams, and experiment results. These are not in this repository.

## Safety and statistical scope

The positions in Table III refer to the experimental frame and robot configuration. They **must not** be copied as a universal FR3 safe workspace. A physical robot must still enforce its own calibrated joint, force/torque, collision, contact, time, and emergency-stop limits. The software has no robot driver or live execution path by design. `waypoint_intents` are suitable for offline inspection only.

The paper's gains are statements about its measured experiments, **not** regression tests or results achieved by this software. The demo uses synthetic numbers and cannot validate grasp success. Router replays cannot infer physical outcome under unexecuted parameter choices.

The `nominal_speed_m_s=0.1` used for dry-run position intents is an **illustrative default**: the paper reports `lambda_v`, not the calibrated nominal speed. `motion_plan_budget_s` is a caller-provided limit for *narrow motion planning*, not the front-end inference time; the default 2 s is an adapter convention.

M1 and M2 use the same fixed low-tier profile; M2 and M3 use the same guarded anchor. `mode="M3"` is the default. The code does not reconstruct original hardware experiments.

Version/profile changes must be audited, retested offline, and validated with real hardware safety review prior to any integration.
