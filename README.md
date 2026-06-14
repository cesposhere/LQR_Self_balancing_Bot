# Self-Balancing Robot Simulation & Teleoperation

> **Executive Summary:** Developed self-balancing robot simulation via CoppeliaSim Python API, implementing LQR velocity control and dynamic teleoperation constraints.

## 📌 Project Overview
This repository contains the URDF models, CoppeliaSim simulation scene, and Python control scripts for a two-wheeled self-balancing robot. Serving as an independent technical project, it demonstrates robust closed-loop control and interactive teleoperation. The system actively maintains equilibrium utilizing a custom Linear Quadratic Regulator (LQR) while allowing real-time keyboard inputs, safeguarded by dynamic kinematic constraints to prevent tipping under extreme maneuvers.

## 🛠️ Technical Stack
* **Simulation Environment:** CoppeliaSim (Gazebo alternative)
* **Integration:** CoppeliaSim Python Remote API (ZeroMQ)
* **Languages:** Python
* **Control Theory:** Linear Quadratic Regulator (LQR), State Estimation, World-to-Local Transformations

## ⚙️ Controller Architecture
The control law diverges from standard torque-based LQR by commanding direct joint velocities, providing precise tracking of teleoperation setpoints.
* **State Vector:** Real-time state estimation constructs a 4-state matrix `[x, x_dot, theta, theta_dot]`. Linear and angular velocities are mapped from world frames to local body frames via inverse transformation matrices.
* **Optimal Gain Matrix (K):** Tuned for aggressive stabilization: `[-141.4214, -87.1545, 253.5076, 11.3576]`.
* **Actuator Saturation:** Target motor velocities are rigidly clamped to `±20.0` rad/s.

## 🎮 Teleoperation & Safety Constraints
To bridge the gap between theoretical LQR stabilization and practical user control, the system implements advanced safety interventions overriding user commands when necessary:
* **Max Lean Angle Lock:** Blocks forward/backward position setpoint accumulation if the robot's pitch exceeds ~17° (`0.3` rad).
* **Setpoint Bleeding:** If pitch enters the critical threshold (>11°), the system actively bleeds the accumulated setpoint back toward zero at a rate of `0.3` units/sec to autonomously recover stability.
* **Dynamic Turn Scaling:** Differential drive turning velocities are proportionally scaled down as the tilt angle approaches the maximum turn threshold, preventing centrifugal force from inducing a tip-over state.

## 🚀 Quick Start & Usage

### 1. Mesh Configuration
For the URDF to render correctly in 3D viewers, ensure all exported `.dae` mesh files are placed in the `urdf/meshes/` directory. The `.urdf` file utilizes relative paths to ensure seamless cross-platform compatibility.

### 2. Running the Simulation
1. Launch CoppeliaSim and open the `.ttt` scene file.
2. Run the main control script:
```bash
python task1b_solution.py
```
To get the Q matrix Use LQR_q_solve.py 
