# Inertial-Control-Sandbox-ICS-
#Flavortown – 3D Rigid Body Simulator & Attitude Control Sandbox

Status: Prototype (Initial version complete, Day 3 – 25/12/25)

Overview

Flavortown is a Python-based 3D rigid body simulator designed to experiment with attitude control, sensor simulation, and control algorithms (PID + attitude control). It allows testing of angular velocity tracking and quaternion-based orientation management in a fully simulated environment, providing a flexible tool for hobbyists, researchers, and DIY robotics enthusiasts.

This platform is suitable for learning, prototyping, and algorithm testing for drones, USVs, and other robotic systems.

Features

3D rigid body dynamics with quaternion-based attitude representation.

Simulated IMU with configurable noise and bias.

Kalman filter for angular velocity estimation.

Hybrid PID + Attitude control loop for tracking desired orientations.

Adjustable torque and angular velocity constraints.

Real-time logging of angular velocities, torques, and quaternions.

Configurable simulation timestep for high-precision experiments.

Installation

Clone the repository:

git clone https://github.com/yourusername/flavortown.git
cd flavortown


Install dependencies (Python 3.10+ recommended):

pip install numpy


(Additional packages may be required if extending the simulator.)

Usage

Run the main simulation script:

python main.py


Simulation prints logs every 10 steps:

STEP 000 | OMEGA: [...] | OMEGA_REF: [...] | TORQUE: [...] | Q: [...]


Parameters you can modify:

Q_REF: Desired quaternion orientation.

OMEGA_REF: Reference angular velocity.

PID gains: kp, kd, ki.

Torque/omega limits: can be removed for unrestricted simulation.

STEPS and DT: Simulation duration and timestep.

Code Structure
flavortown/
│
├─ main.py               # Main simulation loop
├─ system/
│  └─ rigid_body_nd.py   # Rigid body dynamics engine
├─ sensors/
│  └─ gyro_nd.py         # IMU simulation
├─ estimators/
│  └─ kalman_nd.py       # Kalman filter for angular velocity
├─ controllers/
│  ├─ pid.py             # PID controller implementation
│  └─ attitude.py        # Attitude controller (quaternion → ω_ref)
└─ math_utils/
   └─ quaternion.py      # Quaternion math utilities

Example Output
STEP 000 | OMEGA: [ 0.0035 -0.0015  0.0014] | OMEGA_REF: [-0.01  0.01 -0.01] | TORQUE: [0.35 -0.15 0.14] | Q: [0.9933 0.1013 -0.0506 0.0202]
STEP 010 | OMEGA: [ 0.0543 -0.0365  0.0175] | OMEGA_REF: [-0.11 0.11 -0.11] | TORQUE: [0.70 -0.38 0.17] | Q: [0.9931 0.1027 -0.0516 0.0207]
...

Applications

Prototype and test flight controllers for DIY drones.

Experiment with submarine and USV attitude control.

Study PID tuning, sensor fusion, and quaternion math.

Educational tool for robotics, aerospace, and mechanical engineering.

Notes

The simulation currently applies torque and angular velocity limits to maintain stability.

Future versions may include hydrodynamics, aerodynamics, and hardware-in-the-loop (HIL) support.

Platform is not yet intended for real-world deployment, only for algorithm development and experimentation.

License

MIT License – feel free to use, modify, and extend.
