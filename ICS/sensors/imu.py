import random

class Gyro1D:
    """
    Giroscópio 1D com ruído gaussiano e bias constante.
    """
    def __init__(self, noise_std=0.05, bias=0.0):
        self.noise_std = noise_std
        self.bias = bias

    def read(self, true_omega):
        noise = random.gauss(0.0, self.noise_std)
        return true_omega + self.bias + noise
