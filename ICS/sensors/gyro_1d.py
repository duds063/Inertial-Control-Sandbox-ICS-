import random

class Gyro1D:
    def __init__(self, noise_std=0.05, bias=0.0):
        self.sigma = noise_std
        self.bias = bias

    def read(self, true_omega):
        noise = random.gauss(0, self.sigma)
        return true_omega + self.bias + noise