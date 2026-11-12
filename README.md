# MR Damper Semi-Active Suspension Simulator (MATLAB, Option A)

Quarter-car (2-DOF) model: passive damper vs MR semi-active (Skyhook, PID), same road for all.

## Run
1. Open MATLAB in this folder (R2018b or newer, no toolboxes needed).
2. `main`       -> validation checks, bump + random-road tables, figures, results.mat
3. `dashboard`  -> animated side-by-side simulator (choose road/controller, Run, Play)

## Files
| File | Role |
|---|---|
| vehicle_parameters.m | every parameter (slides 4-7) |
| road_profile.m | bump / ISO 8608 random road (half-step grid, seeded) |
| mr_damper_model.m, damping_to_current.m | c(I) map, F = c*vrel, current saturation |
| skyhook_controller.m, pid_controller.m | the two controllers |
| simulate_suspension.m | RK4 closed loop for passive / skyhook / pid |
| performance_metrics.m, compare_metrics.m, print_comparison.m | RMS, peaks, settling, % improvement |
| animation.m | draws/animates one quarter-car |
| dashboard.m | GUI |

## Reference results (my Python port of the same equations)
Bump (slide 6): peak body disp 39.1 -> 27.5 mm, peak accel 5.91 -> 4.29 m/s^2, settling 1.39 -> 0.39 s.
Random road, Skyhook (slide 7): RMS accel about -20%, tyre deflection about +19% worse.
PID (Kp=6000, Ki=20000, Kd=200): bump 24.5 mm / 4.29 m/s^2 / 0.54 s; random RMS accel about -20%, tyre about +18% worse.

## Limitations
- MR damper is a linear viscous model (no hysteresis, saturation or Bouc-Wen dynamics); set `p.tau_mr = 0.01` to add actuator lag.
- Controllers use ideal velocity feedback.
- Animation exaggerates displacements (x4) for visibility.
- PID gains were tuned on a small grid; PID essentially behaves like a Skyhook variant here, so it does not fix the tyre-deflection trade-off. Next step: hybrid Skyhook-groundhook.
