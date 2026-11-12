# SuspensionDna
### MR Damper Semi-Active Suspension Simulator

<p align="center">
  **An interactive MATLAB-based simulation of conventional and magnetorheological semi-active vehicle suspension systems.**
</p>

---

## About the Project

**SuspensionDna** is an engineering simulation project focused on investigating the performance of Magnetorheological (MR) dampers in semi-active vehicle suspension systems.

The project uses a **2-DOF quarter-car model** to compare conventional passive suspension with MR-damper-based semi-active suspension under different road conditions.

By implementing Skyhook and PID-like control strategies, the simulator investigates how variable damping can improve ride comfort and suspension performance.

The project also features an animated engineering visualization that demonstrates the dynamic behaviour of both suspension systems.

## Key Features

- 2-DOF quarter-car mathematical model.
- Conventional passive suspension modelling.
- Current-dependent MR damper modelling.
- Continuous Skyhook control strategy.
- PID-like semi-active control strategy.
- Single bump road excitation.
- ISO 8608 random road generation.
- Numerical simulation using fourth-order Runge-Kutta (RK4).
- Passive vs semi-active performance comparison.
- RMS and peak performance calculations.
- Interactive MATLAB dashboard.
- Synchronized 2D quarter-car animation.

## System Overview

The quarter-car model consists of two degrees of freedom:

- **Sprung mass:** Represents the vehicle body.
- **Unsprung mass:** Represents the wheel assembly.

The suspension spring and damper connect the two masses, while tyre stiffness connects the wheel to the road.

The MR damper changes its damping coefficient according to the applied coil current.

### Suspension Comparison

| Parameter | Passive Suspension | MR Semi-Active Suspension |
|---|---|---|
| Damping | Fixed | Variable |
| Control | None | Skyhook / PID-like |
| Coil current | Not required | 0–2 A |
| Road adaptability | Limited | Adaptive |
| Energy requirement | No external control power | Low-power control |

## Mathematical Model

The quarter-car system is governed by two equations of motion.

**Sprung mass:**

\[
m_s\ddot z_s=-k_s(z_s-z_u)-F_{MR}
\]

**Unsprung mass:**

\[
m_u\ddot z_u=k_s(z_s-z_u)+F_{MR}-k_t(z_u-z_r)
\]

The simplified MR damper model is:

\[
F_{MR}=c_{MR}(I)(\dot z_s-\dot z_u)
\]

The damping coefficient is mapped linearly to coil current:

\[
c_{MR}(I)=c_{min}+
(c_{max}-c_{min})\frac{I}{I_{max}}
\]

This is a control-oriented approximation rather than a fully nonlinear physical MR damper model.

## Simulation Parameters

| Parameter | Value |
|---|---:|
| Sprung mass | 300 kg |
| Unsprung mass | 40 kg |
| Suspension stiffness | 18,000 N/m |
| Tyre stiffness | 180,000 N/m |
| Minimum MR damping | 800 N·s/m |
| Maximum MR damping | 5,000 N·s/m |
| Maximum coil current | 2 A |
| Passive damping | 1,500 N·s/m |
| Vehicle speed | 20 m/s |
| Bump height | 0.05 m |
| Bump duration | 0.2 s |

## Controllers

### 1. Skyhook Controller

Implements a continuous saturated Skyhook control strategy that adjusts the MR damper's damping coefficient based on sprung-mass velocity and relative suspension velocity.

### 2. PID-like Controller

Uses a desired damping force based on proportional, integral and derivative terms, subject to the physical constraints of a semi-active damper.

Both controllers are compared against the conventional passive suspension.

## Road Excitation

Two road profiles are implemented:

**1. Single Bump**

A 5 cm half-sine bump lasting 0.2 seconds, beginning at 1 second.

**2. Random Road**

A stochastic road profile synthesized using spatial-frequency components based on an ISO 8608 road roughness model.

Multiple random seeds are used to evaluate performance across different road realizations.

## Performance Evaluation

The simulation evaluates:

| Metric | Purpose |
|---|---|
| Peak body displacement | Maximum chassis movement |
| Peak body acceleration | Maximum acceleration response |
| RMS body acceleration | Ride comfort indicator |
| Suspension travel | Suspension operating range |
| Tyre deflection | Road-holding indicator |
| Damper force | Damping performance |
| Settling time | Time required for the body response to settle |

Percentage improvement is calculated relative to passive suspension.

Positive improvement indicates a reduction in the corresponding lower-is-better metric.

## Reference Simulation Results

The following are preliminary reference results documented in the project files. They are from a Python port of the same equations and should be verified against the MATLAB implementation.

### Bump Test

| Metric | Passive | Skyhook (MR) |
|---|---:|---:|
| Peak body displacement | 39.1 mm | 27.5 mm |
| Peak body acceleration | 5.91 m/s² | 4.29 m/s² |
| Settling time | 1.39 s | 0.39 s |

### Random Road Test

Preliminary Skyhook results indicate approximately:

- 20% reduction in RMS body acceleration.
- 19% deterioration in tyre deflection.

These results demonstrate the ride-comfort versus road-holding trade-off that requires further controller tuning.

**Note:** These are preliminary results, not experimentally validated performance claims.

## Interactive Dashboard

The project includes an interactive MATLAB dashboard featuring:

- Side-by-side passive and MR suspension animations.
- Road profile selection.
- Skyhook and PID-like controller selection.
- Run, Play, Pause and Reset controls.
- Adjustable animation speed.
- Live MR damper current.
- Live damping coefficient and damping force.
- Body displacement response graph.
- Body acceleration response graph.
- Tyre deflection response graph.
- MR current response graph.
- Automatic performance comparison table.

The animation uses the numerical simulation outputs, with displacement visually exaggerated by a factor of four for clarity.

## Project Structure

```text
SuspensionDna/
│
├── src/
│   ├── main.m
│   ├── dashboard.m
│   ├── animation.m
│   ├── vehicle_parameters.m
│   ├── road_profile.m
│   ├── simulate_suspension.m
│   ├── mr_damper_model.m
│   ├── damping_to_current.m
│   ├── skyhook_controller.m
│   ├── pid_controller.m
│   ├── performance_metrics.m
│   ├── compare_metrics.m
│   └── print_comparison.m
│
├── docs/
│
├── results/
│
├── assets/
│
├── README.md
└── .gitignore
```

## Getting Started

### Requirements

- MATLAB R2018b or newer.
- No additional toolboxes required according to the current implementation.

### Installation

1. Clone the repository:

```bash
git clone https://github.com/YOUR_USERNAME/SuspensionDna.git
```

2. Navigate to the source directory:

```bash
cd SuspensionDna/src
```

3. Open the folder in MATLAB.

### Running the Simulation

**Step 1: Run the main simulation**

```matlab
main
```

This performs validation checks, executes bump and random-road simulations, generates comparison tables and plots, and saves simulation results.

**Step 2: Launch the interactive dashboard**

```matlab
dashboard
```

Choose a road profile and controller, click Run, and then Play to observe the synchronized suspension animations.

## Limitations

The current implementation has several limitations:

- The MR damper uses a simplified linear viscous damping model.
- Nonlinear hysteresis is not represented.
- Controllers use ideal velocity feedback.
- PID-like controller gains require further tuning.
- Random-road results show a trade-off between ride comfort and tyre deflection.
- The animation uses exaggerated displacement for visibility.
- Experimental validation has not yet been completed.

## Future Improvements

Potential future development includes:

- Implementation of a nonlinear Bouc-Wen MR damper model.
- Hybrid Skyhook-Groundhook control.
- Improved PID tuning and controller comparison.
- MR damper actuator response modelling.
- ISO 2631 frequency-weighted ride comfort evaluation.
- Enhanced animation and visualization.
- Experimental validation using an MR damper test rig.
- Simulink-based control system representation.

## Contributors

**Abhinav Sharma**

**Ojaswi Rai**

## Project Status

**Under Development**

SuspensionDna is an ongoing engineering project. The mathematical simulation and initial interactive visualization are implemented, while further model refinement, controller tuning and validation remain in progress.

---

*SuspensionDna — Exploring smarter suspension through adaptive damping.*
