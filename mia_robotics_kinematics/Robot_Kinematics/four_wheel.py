import numpy as np
from mia_robotics_kinematics.Robot_Kinematics.kinematics import Kinematics

class FourWheelKinematics(Kinematics):
    def __init__(self, L: float, W: float, R: float):
        super().__init__(L, W, R)

        # For 4-Wheel Omni, L is the distance from robot center to each wheel center.
        self.M_inverse = (1.0 / self.R) * np.array([
            [-np.sin(np.pi / 4.0),       np.cos(np.pi / 4.0),       self.L],
            [-np.sin(3.0 * np.pi / 4.0), np.cos(3.0 * np.pi / 4.0), self.L],
            [-np.sin(5.0 * np.pi / 4.0), np.cos(5.0 * np.pi / 4.0), self.L],
            [-np.sin(7.0 * np.pi / 4.0), np.cos(7.0 * np.pi / 4.0), self.L]
        ], dtype=np.float64)

        self.M_forward = (self.R / 4.0) * np.array([
            [-np.sqrt(2), -np.sqrt(2),  np.sqrt(2),  np.sqrt(2)],
            [ np.sqrt(2), -np.sqrt(2), -np.sqrt(2),  np.sqrt(2)],
            [ 1.0 / self.L, 1.0 / self.L, 1.0 / self.L, 1.0 / self.L]
        ], dtype=np.float64)