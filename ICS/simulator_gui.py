import sys
import numpy as np
import csv
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QPushButton,
    QLabel, QDoubleSpinBox, QHBoxLayout, QGridLayout
)
from PyQt5.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Import your existing simulator
from system.rigid_body_nd import RigidBody3D
from sensors.gyro_nd import GyroND
from estimators.kalman_nd import KalmanND
from controllers.pid import PIDND
from controllers.attitude import AttitudeController
from math_utils.quaternion import quat_norm

DT = 0.01

class SimulatorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Flavortown Simulator GUI")
        self.resize(900, 600)

        # ----------------- Simulator -----------------
        self.rb = RigidBody3D(inertia=np.array([1.0,1.0,1.0]), dt=DT)
        self.rb.q = quat_norm(np.array([0.98,0.1,-0.05,0.02]))

        self.imu = GyroND(dim=3, noise_std=0.05, bias=[0.5,-0.25,0.1])
        self.kf = KalmanND(dim=3, q_omega=1e-3, q_bias=1e-6, r=0.05**2)
        self.attitude = AttitudeController(kp=6.0, kd=1.2)
        self.pid = PIDND(
            kp=[6,6,6], kd=[0.35,0.35,0.35], ki=[0.0,0.0,0.0],
            dt=DT, torque_limit=[1.0,1.0,1.0], d_cutoff=15.0
        )

        self.Q_REF = np.array([1.0,0.0,0.0,0.0])
        self.OMEGA_DES = np.array([0.05,0,0])

        self.step_counter = 0

        # ----------------- GUI -----------------
        layout = QVBoxLayout()

        # Matplotlib figure
        self.fig = Figure()
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.ax_omega = self.fig.add_subplot(211)
        self.ax_torque = self.fig.add_subplot(212)

        self.ax_omega.set_title("Angular Velocity (rad/s)")
        self.ax_torque.set_title("Torque")

        self.omega_history = np.zeros((3,1))
        self.torque_history = np.zeros((3,1))
        self.time_history = [0]

        # Controls
        control_layout = QGridLayout()
        self.kp_spin = [QDoubleSpinBox() for _ in range(3)]
        self.kd_spin = [QDoubleSpinBox() for _ in range(3)]
        self.omega_spin = [QDoubleSpinBox() for _ in range(3)]

        for i in range(3):
            self.kp_spin[i].setRange(0, 50); self.kp_spin[i].setValue(self.pid.kp[i])
            self.kd_spin[i].setRange(0, 50); self.kd_spin[i].setValue(self.pid.kd[i])
            self.omega_spin[i].setRange(-5, 5); self.omega_spin[i].setValue(self.OMEGA_DES[i])
            control_layout.addWidget(QLabel(f"KP{i+1}"),0,i); control_layout.addWidget(self.kp_spin[i],1,i)
            control_layout.addWidget(QLabel(f"KD{i+1}"),2,i); control_layout.addWidget(self.kd_spin[i],3,i)
            control_layout.addWidget(QLabel(f"Ω{i+1}"),4,i); control_layout.addWidget(self.omega_spin[i],5,i)

        layout.addLayout(control_layout)

        # Start/Stop button
        self.btn_start = QPushButton("Start Simulation")
        self.btn_start.clicked.connect(self.toggle_simulation)
        layout.addWidget(self.btn_start)

        self.setLayout(layout)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.simulation_step)
        self.running = False

        # CSV logging
        self.csv_file = open("sim_data.csv","w",newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time","omega_x","omega_y","omega_z","torque_x","torque_y","torque_z"])

    def toggle_simulation(self):
        if self.running:
            self.timer.stop()
            self.btn_start.setText("Start Simulation")
            self.running = False
        else:
            self.timer.start(int(DT*1000))
            self.btn_start.setText("Stop Simulation")
            self.running = True

    def simulation_step(self):
        # Update PID and desired values
        self.pid.kp = [spin.value() for spin in self.kp_spin]
        self.pid.kd = [spin.value() for spin in self.kd_spin]
        self.OMEGA_DES = np.array([spin.value() for spin in self.omega_spin])

        omega_true = self.rb.omega.copy()
        z = self.imu.read(omega_true)
        self.kf.predict()
        omega_est, bias_est = self.kf.update(z)
        omega_est = np.array(omega_est)

        # Attitude control
        omega_ref = self.attitude.step(self.Q_REF, self.rb.q, omega_est)

        # Rate control
        rate_error = self.OMEGA_DES - omega_est
        torque = np.array(self.pid.step(rate_error, -omega_est))
        self.rb.step(torque)

        # Update history
        self.omega_history = np.hstack([self.omega_history, omega_true.reshape(3,1)])
        self.torque_history = np.hstack([self.torque_history, torque.reshape(3,1)])
        self.time_history.append(self.step_counter*DT)
        self.step_counter += 1

        # Update plots
        self.ax_omega.clear(); self.ax_torque.clear()
        self.ax_omega.plot(self.time_history, self.omega_history[0], label="ωx")
        self.ax_omega.plot(self.time_history, self.omega_history[1], label="ωy")
        self.ax_omega.plot(self.time_history, self.omega_history[2], label="ωz")
        self.ax_omega.legend()
        self.ax_omega.set_title("Angular Velocity (rad/s)")

        self.ax_torque.plot(self.time_history, self.torque_history[0], label="τx")
        self.ax_torque.plot(self.time_history, self.torque_history[1], label="τy")
        self.ax_torque.plot(self.time_history, self.torque_history[2], label="τz")
        self.ax_torque.legend()
        self.ax_torque.set_title("Torque")
        self.canvas.draw()

        # Log to CSV
        self.csv_writer.writerow([self.step_counter*DT, *omega_true, *torque])


if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SimulatorGUI()
    gui.show()
    sys.exit(app.exec_())