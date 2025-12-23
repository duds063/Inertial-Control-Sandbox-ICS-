import numpy as np

class KalmanGyroBias1D:
    """
    Kalman Filter para giroscópio 1D com bias.

    Estado:
        x = [ omega,
              bias ]

    Medição:
        z = omega + bias + v
    """

    def __init__(
        self,
        omega0=0.0,
        bias0=0.0,
        P0=1.0,
        q_omega=1e-3,
        q_bias=1e-6,
        r_meas=0.05**2
    ):
        # Estado
        self.x = np.array([[omega0],
                           [bias0]])

        # Covariância
        self.P = np.eye(2) * P0

        # Modelo
        self.F = np.eye(2)
        self.H = np.array([[1.0, 1.0]])

        # Ruídos
        self.Q = np.array([[q_omega, 0.0],
                           [0.0,     q_bias]])
        self.R = np.array([[r_meas]])

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        z = np.array([[z]])

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P

        return float(self.x[0]), float(self.x[1])