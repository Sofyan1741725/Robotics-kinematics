import numpy as np
from kinematics import kinematics
import matplotlib.pyplot as plt

class MecanumKinematics(kinematics):

    def __init__(self,L,W,R):
        super().__init__(L,W,R)

        self.M_forward = np.array([[R/4,R/4,R/4,R/4],
                                   [-R/4,R/4,R/4,-R/4],
                                   [-R/(4*(L+W)),R/(4*(L+W)),-R/(4*(L+W)),R/(4*(L+W))]])

        self.M_inverse = np.array([[1/R,-1/R,-(L+W)/R],
                                   [1/R,1/R,(L+W)/R],
                                   [1/R,1/R,-(L+W)/R],
                                   [1/R,-1/R,(L+W)/R]])
        


    def inverse(self,Vx: float,Vy: float,Wz: float) -> list:

        # w1> velocity in left wheel 

        w1 = (Vx-Vy-Wz*(self.L+self.W))/self.R
        w2 = (Vx+Vy+Wz*(self.L+self.W))/self.R
        w3 = (Vx+Vy-Wz*(self.L+self.W))/self.R
        w4 = (Vx-Vy+Wz*(self.L+self.W))/self.R


        return [w1,w2,w3,w4]

    def forward(self,W: list) -> tuple:

        w1 = W[0]
        w2 = W[1]
        w3 = W[2]
        w4 = W[3]

        Vx = self.R/4*(w1+w2+w3+w4)
        Vy = self.R/4*(-w1+w2+w3-w4)
        Wz = self.R/(4*(self.L+self.W))*(-w1+w2-w3+w4)



        return Vx,Vy,Wz