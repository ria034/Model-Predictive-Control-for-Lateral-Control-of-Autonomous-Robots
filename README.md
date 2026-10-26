# Model Predictive Control for Lateral Control of Autonomous Robots

Lateral (steering) control of a car-like robot along a reference path. Six
controllers are compared on four driving scenarios, under sensor noise,
model mismatch and actuator delay:

| Controller | File | Idea |
|---|---|---|
| Pure pursuit | `controllers/ctrl_pure_pursuit.m` | Geometric, steers toward a look-ahead point on the path |
| Stanley | `controllers/ctrl_stanley.m` | Geometric, heading error + cross-track error at the front axle |
| LQR | `controllers/ctrl_lqr.m` | Optimal state feedback on the lateral error model, gain-scheduled on speed, with curvature feedforward |
| Linear MPC | `controllers/ctrl_lmpc.m` | One linear model at a fixed design speed (15 m/s) |
| Adaptive MPC | `controllers/ctrl_ampc.m` | Linear time-varying MPC: a model for every horizon step at the speed the car will have there |
| Nonlinear MPC | `controllers/ctrl_nmpc.m` | Real-time iteration: re-linearizes the nonlinear tyre model along its last solution every sample |

All three MPCs preview the path curvature, respect steering-angle and
steering-rate limits, model the steering actuator, and keep the car inside a
lane corridor that narrows around obstacles (soft constraint with a slack
variable).

No toolboxes are needed. The QP solver (`utils/qp_interior_point.m`, a
primal-dual interior-point method) and the LQR (`utils/dlqr_iter.m`) are
included, so everything runs on base MATLAB R2020a or newer.

## Quick start

```matlab
cd Model-Predictive-Control-for-Lateral-Control-of-Autonomous-Robots
setup_paths                 % adds the folders below to the path
run main/main_demo.m        % all controllers on the wet hairpin, with plots
```

One run from the command window:

```matlab
cfg = default_config();
res = run_scenario(cfg, 'obstacles', 'nmpc');
plot_run(res)
animate_run(res)                             % or animate_run(res, 'results/nmpc.mp4')
```

Full benchmark (6 controllers x 4 scenarios x 6 robustness cases):

```matlab
T = run_benchmark();                         % writes results/*.csv, results/figures/*.png
T = run_benchmark('controllers', {'lqr','nmpc'}, 'scenarios', {'lane_change'}, 'seeds', 1:5);
```

## Scenarios (`utils/make_scenario.m`)

| Name | What it tests |
|---|---|
| `lane_change` | ISO 3888-1 style double lane change at a constant 20 m/s |
| `s_curves` | Changing curvature; speed from a 0.6 g lateral-acceleration limit (12 to 19 m/s) |
| `hairpin_wet` | Two tight bends on a wet road (μ = 0.5), braking to 7 m/s and accelerating to 17 m/s |
| `obstacles` | Two-lane road at 15 m/s with three parked cars partly in the lane |

Speed is tracked by a separate controller (profile feedforward plus a
proportional term), so the lateral controllers have to cope with changing
speed.

## Vehicle model (`models/`)

The simulated car is a nonlinear single-track model with Fiala brush tyres
(they saturate at μ·F<sub>z</sub>), a first-order steering actuator with a rate
limit, and longitudinal acceleration limits. It is integrated with RK4 at
2 ms, and the controllers run every 50 ms.

`vehicle_params('sedan')` (default) is a 1600 kg mid-size car.
`vehicle_params('repo')` gives the parameters from the original Simulink
model's Vehicle Body 3DOF block; its tyres are very soft and never come near
saturation, which is why the default is the sedan.

## Robustness cases (`main/apply_case.m`)

| Case | What changes |
|---|---|
| `nominal` | Nothing |
| `noise` | Position σ = 0.1 m, heading σ = 0.3°, yaw rate σ = 0.01 rad/s, lateral velocity σ = 0.05 m/s |
| `mismatch` | Real car 20 % heavier, tyres 20 % softer, 15 % less grip than the controllers assume |
| `delay` | 100 ms steering delay; LQR and the MPCs predict their state forward over the delay |
| `delay_nocomp` | Same delay without that prediction |
| `combined` | Noise, mismatch and delay together |

## Reference results

These are from a Python mirror of this code (same models, gains and
scenarios), included in `tools/python_reference/` so the results can be
checked without MATLAB. MATLAB should give very similar numbers; small
differences come from the random-number generators and floating-point
details. RMS lateral error in metres; `*` means the car left the road.

**Double lane change, 20 m/s**

| | nominal | noise | mismatch | delay | delay, no comp. | combined |
|---|---|---|---|---|---|---|
| pure pursuit | 0.273 | 0.277 | 0.417 | 0.318 | 0.318 | 0.482 |
| Stanley | 0.123 | 0.126 | 0.199 | 0.133 | 0.133 | 0.218 |
| LQR | 0.027 | 0.041 | 0.067 | 0.082 | 1.756* | 0.214 |
| linear MPC | 0.043 | 0.058 | 0.095 | 0.078 | 2.029* | 0.189 |
| adaptive MPC | 0.049 | 0.065 | 0.091 | 0.089 | 1.106* | 0.191 |
| nonlinear MPC | 0.039 | 0.056 | 0.064 | 0.069 | 0.766 | 0.135 |

**S-curves, 12 to 19 m/s**

| | nominal | noise | mismatch | delay | delay, no comp. | combined |
|---|---|---|---|---|---|---|
| pure pursuit | 0.823 | 0.823 | 1.409 | 0.875 | 0.875 | 1.533 |
| Stanley | 0.340 | 0.344 | 0.595 | 0.348 | 0.348 | 0.615 |
| LQR | 0.047 | 0.063 | 0.150 | 0.096 | 0.092 | 0.253 |
| linear MPC | 0.061 | 0.074 | 0.171 | 0.094 | 0.384 | 0.262 |
| adaptive MPC | 0.065 | 0.072 | 0.172 | 0.090 | 0.095 | 0.257 |
| nonlinear MPC | 0.025 | 0.036 | 0.109 | 0.043 | 0.083 | 0.176 |

**Wet hairpin, μ = 0.5**

| | nominal | noise | mismatch | delay | delay, no comp. | combined |
|---|---|---|---|---|---|---|
| pure pursuit | 0.371 | 0.383 | 0.769 | 0.374 | 0.374 | 0.777 |
| Stanley | 0.078 | 0.084 | 0.428 | 0.069 | 0.069 | 0.467 |
| LQR | 0.052 | 0.069 | 0.544 | 0.100 | 0.126 | 0.755 |
| linear MPC | 0.187 | 0.190 | 0.084 | 0.173 | 0.191 | 0.129 |
| adaptive MPC | 0.057 | 0.069 | 0.428 | 0.083 | 0.076 | 0.264 |
| nonlinear MPC | 0.052 | 0.063 | 0.146 | 0.066 | 0.066 | 0.198 |

**Obstacles: minimum clearance in metres (negative = collision)**

| | nominal | noise | mismatch | delay | delay, no comp. | combined |
|---|---|---|---|---|---|---|
| pure pursuit | −1.68 | −1.67 | −1.72 | −1.68 | −1.68 | −1.71 |
| Stanley | −1.63 | −1.62 | −1.66 | −1.64 | −1.64 | −1.64 |
| LQR | −1.63 | −1.59 | −1.63 | −1.63 | −1.63 | −1.59 |
| linear MPC | 0.87 | 0.86 | 0.89 | 0.86 | 0.72 | 1.00 |
| adaptive MPC | 0.87 | 0.86 | 0.89 | 0.86 | 0.62 | 1.00 |
| nonlinear MPC | 0.81 | 0.81 | 0.89 | 0.81 | 0.57 | 0.94 |

What stands out:

- **Delay is the biggest threat to the model-based controllers.** An uncompensated 100 ms steering delay makes LQR and both linear MPCs leave the road in the 20 m/s lane change. Predicting the state over the delay fixes it.
- **The nonlinear MPC is the most accurate in most cases,** especially on the S-curves and under combined disturbances, because its prediction includes tyre saturation.
- **The fixed-speed linear MPC is the weakest MPC on the wet hairpin** (0.19 m against 0.05 m for the adaptive and nonlinear versions), because the speed there ranges from 7 to 17 m/s while its model assumes 15. It does unexpectedly well under mismatch in that scenario, probably because the wrong-speed model happens to offset the softer tyres. That's a coincidence, not robustness.
- **The geometric controllers are the least accurate but barely react to delay,** because their gains are low.
- **Only the MPCs avoid the obstacles.** Pure pursuit, Stanley and LQR only follow the centreline; giving them a planned avoidance path would be a fairer comparison.
- **Lateral error on the obstacle scenario** is measured from the centreline, so it includes the deliberate swerves.

Computation time depends on the PC. In the Python mirror (single core,
uncontended), LQR takes about 2 ms per step, the linear and adaptive MPCs
about 5 ms, and the nonlinear MPC about 20 ms, all within the 50 ms sample time.

## Folder layout

```
config/          default_config.m (every setting), vehicle_params.m
models/          plant (single-track, Fiala tyres, actuator), linear error models, NMPC prediction model
controllers/     the six controllers
utils/           path generation, projection, corridor, QP solver, LQR, discretization
main/            run_scenario, run_benchmark, apply_case, compute_metrics, main_demo
visualization/   plot_run, plot_comparison, plot_benchmark, animate_run
tools/python_reference/   Python mirror used to produce the reference results
results/         created by the scripts
```

## Original Simulink model

`MPC Controller Modelling.slx` and `Cost function Modelling.mlx` are the
first version of this project, using the MPC Toolbox block. That version has
some setup problems: the internal plant model is a double integrator on x, so
steering doesn't affect y or ψ in its predictions; the third reference
channel is speed while the third output is yaw angle; and integrating the
block's body-frame `xdot`/`ydot` doesn't give global position. The
script-based version above replaces it.

## Limitations

- Lateral control only; speed follows a precomputed profile.
- Single-track model without load transfer or combined slip.
- The obstacle corridor picks the passing side from the obstacle position
  (pass on the side with more room); there's no planner choosing between options.
- The nonlinear MPC uses one SQP iteration per sample (real-time iteration),
  not a fully converged nonlinear optimum.
