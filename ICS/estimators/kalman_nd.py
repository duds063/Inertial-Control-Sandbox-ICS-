class KalmanND:
    def __init__(self, dim, q_omega, q_bias, r):
        """
        Kalman ND para estimar omega + bias

        dim      : número de eixos
        q_omega  : variância do ruído do modelo de omega
        q_bias   : variância do ruído do modelo de bias
        r        : variância do ruído de medição
        """

        self.dim = dim

        # Estado
        self.omega = [0.0] * dim
        self.bias = [0.0] * dim

        # Covariâncias (por eixo)
        self.P_omega = [1.0] * dim
        self.P_bias = [1.0] * dim
        self.P_cross = [0.0] * dim  # cov(omega, bias)

        self.q_omega = q_omega
        self.q_bias = q_bias
        self.r = r

    def predict(self):
        for i in range(self.dim):
            self.P_omega[i] += self.q_omega
            self.P_bias[i] += self.q_bias
            # P_cross permanece

    def update(self, z):
        """
        z : list[float] (medição do gyro)
        """

        omega_est = []
        bias_est = []

        for i in range(self.dim):
            # Inovação
            y = z[i] - (self.omega[i] + self.bias[i])

            # Covariância da inovação
            S = (
                self.P_omega[i]
                + self.P_bias[i]
                + 2 * self.P_cross[i]
                + self.r
            )

            # Ganhos de Kalman
            K_omega = (self.P_omega[i] + self.P_cross[i]) / S
            K_bias = (self.P_bias[i] + self.P_cross[i]) / S

            # Atualização do estado
            self.omega[i] += K_omega * y
            self.bias[i] += K_bias * y

            # Atualização das covariâncias
            P_omega = self.P_omega[i]
            P_bias = self.P_bias[i]
            P_cross = self.P_cross[i]

            self.P_omega[i] = P_omega - K_omega * (P_omega + P_cross)
            self.P_bias[i] = P_bias - K_bias * (P_bias + P_cross)
            self.P_cross[i] = (
                P_cross
                - K_omega * (P_bias + P_cross)
            )

            omega_est.append(self.omega[i])
            bias_est.append(self.bias[i])

        return omega_est, bias_est