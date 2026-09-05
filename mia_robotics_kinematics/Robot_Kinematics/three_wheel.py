import numpy as np
from mia_robotics_kinematics.Robot_Kinematics.kinematics import Kinematics

class ThreeWheelKinematics(Kinematics):
    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)

        # Wheels at angles: 90 deg, 210 deg, 330 deg
        self.M_inverse = (1.0 / self.R) * np.array([
            [-np.sin(np.pi / 2.0),        np.cos(np.pi / 2.0),        self.L],
            [-np.sin(7.0 * np.pi / 6.0),  np.cos(7.0 * np.pi / 6.0),  self.L],
            [-np.sin(11.0 * np.pi / 6.0), np.cos(11.0 * np.pi / 6.0), self.L],
            [0.0,                         0.0,                        0.0]
        ], dtype=np.float64)

        # Exact Analytical Inverse Matrix (Pseudo-Inverse of 3-Wheel Omni)
        self.M_forward = (self.R / 3.0) * np.array([
            [-2.0,          1.0,           1.0,          0.0],
            [ 0.0,         -np.sqrt(3),    np.sqrt(3),   0.0],
            [ 1.0 / self.L, 1.0 / self.L,  1.0 / self.L, 0.0]
        ], dtype=np.float64)