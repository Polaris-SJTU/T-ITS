import numpy as np


def get_rotation_matrix(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1],
    ])


def remap_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def constraint(value, lower, upper):
    return lower if value < lower else (upper if value > upper else value)


def map_value(value, lower1, upper1, lower2, upper2):
    value = constraint(value, lower1, upper1)
    value = (value - lower1) / (upper1 - lower1) * (upper2 - lower2) + lower2
    return constraint(value, lower2, upper2)


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.e = 0.0
        self.d_e = 0.0
        self.e_sum = 0.0

    def output(self, e):
        self.e_sum += e
        self.d_e = e - self.e
        self.e = e

        return self.kp * e + self.ki * self.e_sum + self.kd * self.d_e
