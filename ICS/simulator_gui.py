import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from system.rigid_body_nd import RigidBody3D
from sensors.gyro_nd import GyroND
from estimators.kalman_nd import KalmanND
from controllers.pid import PIDND
from controllers.attitude import AttitudeController
from math_utils.quaternion import quat_norm


class SimulatorGUI:
    def __init__(self, root):
        self.root = root
        root.title("Inertial Control Sandbox")

        # ------------------------
        # Simulation parameters
        # ------------------------
        self.dt = 0.01
        self.time = 0.0

        # ------------------------
        # System & controller
        # ------------------------
        self.inertia = np.array([1.0, 1.0, 1.0])

        self.body = RigidBody3D(
            inertia=self.inertia,
            dt=self.dt
        )
        self.body.q = quat_norm(np.array([1.0, 0.0, 0.0, 0.0]))

        # Sensors and estimators
        self.imu = GyroND(dim=3, noise_std=0.05, bias=[0.5, -0.25, 0.1])
        self.kf = KalmanND(dim=3, q_omega=1e-3, q_bias=1e-6, r=0.05**2)

        # Controllers
        self.attitude = AttitudeController(kp=6.0, kd=1.2)
        
        self.pid = PIDND(
            kp=[6, 6, 6],
            kd=[0.4, 0.4, 0.4],
            ki=[0.0, 0.0, 0.0],
            dt=self.dt,
            torque_limit=[1.0, 1.0, 1.0]
        )

        # Reference tracking
        self.omega_ref_prev = np.zeros(3)
        self.q_ref = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega_ref_max = 0.5
        self.slew_rate = 1.0
        
        # Ramp time for reaching target omega
        self.omega_target = np.zeros(3)
        self.omega_ramp_start_time = 0.0

        # ------------------------
        # History
        # ------------------------
        self.t_history = []
        self.omega_history = []
        self.torque_history = []

        # ------------------------
        # GUI Controls
        # ------------------------
        self.controls = ttk.Frame(root)
        self.controls.pack(side=tk.LEFT, fill=tk.Y, padx=10)

        def labeled_control(label, from_, to_, default):
            ttk.Label(self.controls, text=label).pack(anchor="w")

            var = tk.DoubleVar(value=default)

            frame = ttk.Frame(self.controls)
            frame.pack(fill=tk.X)

            scale = ttk.Scale(frame, from_=from_, to=to_, variable=var)
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)

            entry = ttk.Entry(frame, width=7, textvariable=var)
            entry.pack(side=tk.RIGHT)

            return var

        self.kp = labeled_control("Kp", 0, 10, 6)
        self.kd = labeled_control("Kd", 0, 5, 0.4)
        self.ki = labeled_control("Ki", 0, 5, 0.0)

        self.omega_ref_x = labeled_control("ω_ref X", -1, 1, 0)
        self.omega_ref_y = labeled_control("ω_ref Y", -1, 1, 0)
        self.omega_ref_z = labeled_control("ω_ref Z", -1, 1, 0.5)

        self.torque_limit = labeled_control("Torque limit (Nm)", 0.1, 5, 1)
        self.attitude_kp = labeled_control("Attitude Kp", 0, 20, 6)
        self.attitude_kd = labeled_control("Attitude Kd", 0, 5, 1.2)
        self.slew_rate = labeled_control("Slew rate (rad/s²)", 0, 5, 1)
        self.ramp_time = labeled_control("Ramp time (s)", 0, 5, 0.5)

        ttk.Button(
            self.controls,
            text="Reset",
            command=self.reset
        ).pack(pady=10)

        # ------------------------
        # Plot
        # ------------------------
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(6, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.root.after(10, self.simulation_step)

    # ------------------------
    def reset(self):
        self.time = 0.0
        self.body.reset()
        self.body.q = quat_norm(np.array([1.0, 0.0, 0.0, 0.0]))
        self.pid.reset()
        self.kf = KalmanND(dim=3, q_omega=1e-3, q_bias=1e-6, r=0.05**2)
        self.attitude.reset()
        self.omega_ref_prev = np.zeros(3)
        self.omega_target = np.zeros(3)
        self.omega_ramp_start_time = 0.0
        self.t_history.clear()
        self.omega_history.clear()
        self.torque_history.clear()

    # ------------------------
    def simulation_step(self):
        # Update controller parameters
        kp = self.kp.get()
        kd = self.kd.get()
        ki = self.ki.get()

        self.pid.kp = np.array([kp, kp, kp])
        self.pid.kd = np.array([kd, kd, kd])
        self.pid.ki = np.array([ki, ki, ki])
        self.pid.torque_limit = np.array([
            self.torque_limit.get(),
            self.torque_limit.get(),
            self.torque_limit.get()
        ])

        # Update attitude controller
        self.attitude.kp = self.attitude_kp.get()
        self.attitude.kd = self.attitude_kd.get()
        slew_rate = self.slew_rate.get()
        ramp_time = self.ramp_time.get()
        
        # Get new omega target
        omega_target_new = np.array([
            self.omega_ref_x.get(),
            self.omega_ref_y.get(),
            self.omega_ref_z.get()
        ])
        
        # Check if target changed
        if not np.allclose(omega_target_new, self.omega_target):
            self.omega_target = omega_target_new
            self.omega_ramp_start_time = self.time

        # Sensor reading
        omega_true = self.body.omega.copy()
        z = self.imu.read(omega_true)
        
        # Kalman filter estimation
        self.kf.predict()
        omega_est, _ = self.kf.update(z)
        omega_est = np.array(omega_est)

        # Attitude control
        omega_ref_cmd = self.attitude.step(self.q_ref, self.body.q, omega_est)
        omega_ref_cmd = np.clip(omega_ref_cmd, -self.omega_ref_max, self.omega_ref_max)
        
        # Apply ramp time to reach target omega
        if ramp_time > 0:
            time_since_change = self.time - self.omega_ramp_start_time
            if time_since_change < ramp_time:
                # Linear ramp from current to target
                progress = time_since_change / ramp_time
                omega_ref = self.omega_ref_prev + progress * (self.omega_target - self.omega_ref_prev)
            else:
                # Target reached
                omega_ref = self.omega_target.copy()
        else:
            # No ramp time, use target directly
            omega_ref = self.omega_target.copy()
        
        self.omega_ref_prev = omega_ref.copy()


        torque = kp * (omega_ref - omega_est) - kd * omega_est
        
        # Convert to numpy array if needed
        if isinstance(torque, list):
            torque = np.array(torque)

        # Dynamics
        self.body.step(torque)

        # History
        self.time += self.dt
        self.t_history.append(self.time)
        self.omega_history.append(self.body.omega.copy())
        self.torque_history.append(torque.copy())

        self.update_plot()
        self.root.after(10, self.simulation_step)

    # ------------------------
    def update_plot(self):
        if len(self.t_history) < 2:
            return

        self.ax1.clear()
        self.ax2.clear()

        omega_hist = np.array(self.omega_history)
        torque_hist = np.array(self.torque_history)

        self.ax1.plot(self.t_history, omega_hist)
        self.ax1.set_title("Angular Velocity (rad/s)")
        self.ax1.grid()

        self.ax2.plot(self.t_history, torque_hist)
        self.ax2.set_title("Torque (Nm)")
        self.ax2.grid()

        self.canvas.draw()


if __name__ == "__main__":
    root = tk.Tk()
    app = SimulatorGUI(root)
    root.mainloop()