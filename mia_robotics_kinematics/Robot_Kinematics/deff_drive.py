import numpy as np
from kinematics import kinematics
import matplotlib.pyplot as plt

class DiffDriveKinematics(kinematics):

    def __init__(self,L,W,R):
        super().__init__(L,W,R)

        self.M_forward = np.array([[self.R/4,self.R/4,self.R/4,self.R/4],
                                   [0,0,0,0],
                                   [-self.R/(2*W),self.R/(2*W),self.R/(2*W),-self.R/(2*W)]])

        self.M_inverse = np.array([[1/R,0,-self.W/(2*R)],
                                   [1/R,0,self.W/(2*R)],
                                   [1/R,0,self.W/(2*R)],
                                   [1/R,0,-self.W/(2*R)]])
        


    def inverse(self,Vx,Vy,Wz):

        # i considered the robot has two wheels
        # w1> velocity in left wheel 

        left=(Vx-Wz*self.W/2)/self.R
        right=(Vx+Wz*self.W/2)/self.R

        w1=left
        w2=right
        w3=right
        w4=left

        return [w1,w2,w3,w4]

    def forward(self,W):

        w1 = W[0]
        w2 = W[1]
        w3 = W[2]
        w4 = W[3]

        left = (w1+w4)/2
        right = (w2+w3)/2

        Vx = self.R*(right+left)/2
        Vy = 0
        Wz = self.R*(right-left)/self.W

        return Vx,Vy,Wz