# Inertial Control Sandbox (ICS)

The **Inertial Control Sandbox (ICS)** is a lightweight physics-based simulation
environment for developing, testing, and visualizing attitude and angular-rate
controllers for rigid bodies such as drones, satellites, and underwater vehicles.

It was designed to bridge the gap between theoretical control design and
hands-on DIY experimentation.

---

## 🚀 Features

- 3D rigid-body rotational dynamics
- Multi-axis PID controller (P / PI / PID)
- Torque saturation and angular velocity limits
- Actuator reaction delay (reaction time)
- Real-time interactive GUI (Tkinter + Matplotlib)
- Live tuning of:
  - Kp, Ki, Kd
  - Angular rate references
  - Torque and omega limits
  - Reaction time
- Time-domain plots for angular velocity and torque
- Modular architecture (controllers, system, estimators)

---

## 🧠 What This Simulator Does
ICS simulates a **rigid body under rotational control**:

1. A desired angular velocity (`ω_ref`) is defined.
2. A PID controller computes the required control torque.
3. The torque is optionally delayed (reaction time).
4. The rigid body integrates angular acceleration.
5. Physical constraints (torque / omega limits) are enforced.
6. Results are plotted in real time.

The system is deterministic, discrete-time, and suitable for
controller prototyping and tuning.
--- 
## 📁 Project Structure
ICS/
├── system/
│ └── rigid_body_nd.py # Rigid body rotational dynamics
│
├── controllers/
│ └── pid.py # Multi-axis PID controller
│
├── estimators/
│ └── kalman_bias_nd.py # (Optional) Kalman filter with bias estimation
│
├── sensors/
│ └── gyro_nd.py # Gyroscope model with noise/bias
│
├── simulator_gui.py # Interactive GUI application
├── main.py # Headless simulation / experiments
└── README.md
---
## 🖥️ Running the GUI

```bash
python simulator_gui.py
```
The GUI allows real-time tuning of control parameters and immediate visualization
of system response.

## 🎯 Intended Use Cases
DIY drone flight controller prototyping

USV / ROV attitude control testing

Control theory experimentation

PID tuning intuition development

Educational demonstrations

Rapid controller debugging without hardware risk

## ⚠️ What This Is NOT
 - Not a full flight stack (no navigation, no translation)

 - Not a high-fidelity CFD simulator

 - Not hardware-in-the-loop (yet)

ICS focuses strictly on rotational dynamics and control.

## 🧩 Design Philosophy
 - Physics first: dynamics are explicit and transparent

 - Modular components: controllers, sensors, estimators are swappable

 - Interactive experimentation over black-box automation

 - DIY-friendly: minimal dependencies, readable code

## 🛠️ Future Improvements (Planned)
 - Full separation between core simulation and GUI

 - Quaternion-based attitude control mode

 - Sensor noise toggles via GUI

 - Data export (CSV) from GUI

 - Alternative controllers (LQR, MPC)

 - Hardware-in-the-loop support

##  📜 License
This project is intended for educational and experimental use.
Feel free to fork, modify, and learn from it.
