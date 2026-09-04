import numpy as np 

# base class, has repeated attributes 

class kinematics:
    def __init__(self,L,W,R):

        self.L= L
        self.W= W
        self.R= R

        self.M_forward = None 
        self.M_revesr = None 

    # function declaration, it will be used in another classes,
    # this function will convert velocities to wheel_angular_speed    
    def inverse(self,Vx,Vy,Wz):
        pass

    # this function will convert wheel_rotational_speed to velocities
    def forward(self, W):
        pass