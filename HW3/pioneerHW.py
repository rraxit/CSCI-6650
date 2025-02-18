#python
from math import atan2
import numpy as np
    
    
def sysCall_init():
    sim = require('sim')
    self.motorLeft=sim.getObject("/PioneerP3DX/leftMotor")
    self.motorRight=sim.getObject("/PioneerP3DX/rightMotor")
    self.Kp_rho = 0.75
    self.Kp_alpha = 1.8
    self.Kp_beta = 0.6
    self.length = 0.381
    self.radius = 0.195/2.0
    

def sysCall_actuation():
    # put your actuation code here
    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
            
        rho = np.hypot(x_diff, y_diff)
        alpha = angle_mod(np.arctan2(y_diff, x_diff) - theta)
        beta = angle_mod(theta_goal - theta - alpha)
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w
        
    target = getTargetPosition('/Cuboid')
    robot = getTargetPosition('/PioneerP3DX')
    x_diff = target[0] - robot[0]
    y_diff = target[1] - robot[1]
    orientation = getTargetOrientation('/PioneerP3DX')
        
    theta_goal = atan2(y_diff, x_diff)
    rho, v, w = calc_control_command(self, x_diff, y_diff, orientation[-1], theta_goal)
    print(f'v={v},w = {w}')
    vLeft,vRight = uniCyclevel(v,w)
    sim.setJointTargetVelocity(self.motorLeft,vLeft)
    sim.setJointTargetVelocity(self.motorRight,vRight)
    

def sysCall_sensing():
        # put your sensing code here
        pass

def sysCall_cleanup():
        # do some clean-up here
        pass

def angle_mod(x, zero_2_2pi=False, degree=False):
    
    if isinstance(x, float):
        is_float = True
    else:
        is_float = False

    x = np.asarray(x).flatten()
    if degree:
        x = np.deg2rad(x)

    if zero_2_2pi:
        mod_angle = x % (2 * np.pi)
    else:
        mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

    if degree:
        mod_angle = np.rad2deg(mod_angle)

    if is_float:
        return mod_angle.item()
    else:
        return mod_angle
        
def getTargetPosition(name):
    objHandle = sim.getObject(name)
    pos = sim.getObjectPosition(objHandle, -1)
    return pos
    
def getTargetOrientation(name):
        
    objHandle = sim.getObject(name)
    q = sim.getObjectOrientation(objHandle, -1)
    return q

def uniCyclevel(v,w):
    
    vR = (2*v + w * self.length) / (2 * self.radius)
    vL = (2*v - w * self.length) / (2 * self.radius)
    return vL,vR
        

        
        
    

    # See the user manual or the available code snippets for additional callback functions and details
