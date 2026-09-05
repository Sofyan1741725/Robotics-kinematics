import numpy as np 

# base class, has repeated attributes 

class kinematics:
    def __init__(self,L: float,W: float,R: float):

        self.L= float(L)
        self.W= float(W)
        self.R= float(R)

        self.M_forward = None 
        self.M_inverse = None 
    # function declaration, it will be used in another classes,
    # this function will convert velocities to wheel_angular_speed    
    def inverse(self, vx: float, vy: float, wz: float) -> list:
        if self.M_inverse is None:
            raise NotImplementedError("M_inverse is not defined.")
        
        v_body = np.array([vx, vy, wz], dtype=np.float64)
        w_wheels = np.dot(self.M_inverse, v_body)
        return w_wheels.tolist()

    # this function will convert wheel_rotational_speed to velocities
    def forward(self, w: list) -> tuple:
        if self.M_forward is None:
            raise NotImplementedError("M_forward is not defined.")
        
        w_wheels = np.array(w, dtype=np.float64)
        v_body = np.dot(self.M_forward, w_wheels)
        return float(v_body[0]), float(v_body[1]), float(v_body[2])