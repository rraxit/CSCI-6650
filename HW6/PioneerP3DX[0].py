#python
from math import atan2
import numpy as np
    
    
def sysCall_init():
    sim = require('sim')
    self.robot1 = sim.getObjectHandle('/PioneerP3DX[0]')
    self.motorLeft=sim.getObject("/PioneerP3DX[0]/leftMotor")
    self.motorRight=sim.getObject("/PioneerP3DX[0]/rightMotor")
    self.Kp_rho = 0.75
    self.Kp_alpha = 1.8
    self.Kp_beta = 0.6
    self.length = 0.381
    self.radius = 0.195/2.0
    self.obstacles = sim.createCollection(0)
    self.noDetectionDist = 0.5
    self.maxDetectionDist = 0.2
    sim.addItemToCollection(self.obstacles, sim.handle_all, -1, 0)
    sim.addItemToCollection(self.obstacles, sim.handle_tree, self.robot1, 1)
    self.detect = [0] * 16
    self.braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6] + [0] * 8
    self.braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2] + [0] * 8
    self.v0 = 0.05
    self.usensors = []
    self.vLeft = 0
    self.vRight = 0
    for i in range(16):
        self.usensors.append(sim.getObjectHandle(f'/PioneerP3DX[0]/ultrasonicSensor[{i}]'))
        sim.setObjectInt32Param(self.usensors[i], sim.proxintparam_entity_to_detect, self.obstacles)
        #self.usensors.append(self.usensor_handle)
    
def sysCall_actuation():
    # put your actuation code here
    for i in range(16):
        res, dist, *_ = sim.readProximitySensor(self.usensors[i])
        if res > 0 and dist < self.noDetectionDist:
            if dist < self.maxDetectionDist:
                dist = self.maxDetectionDist
            self.detect[i] = 1 - ((dist - self.maxDetectionDist) / (self.noDetectionDist - self.maxDetectionDist))
        else:
            self.detect[i] = 0

    # Default behavior

    
    if all(x == 0 for x in self.detect):
        normal_behavior()
    
    else:
    # Avoidance behavior
        for i in range(16):
            self.vLeft += self.braitenbergL[i] * self.detect[i]
            self.vRight += self.braitenbergR[i] * self.detect[i]
        sim.setJointTargetVelocity(self.motorLeft, self.vLeft)
        sim.setJointTargetVelocity(self.motorRight, self.vRight)

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
        
def normal_behavior():
    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
            
        rho = np.hypot(x_diff, y_diff)
        alpha = angle_mod(np.arctan2(y_diff, x_diff) - theta)
        beta = angle_mod(theta_goal - theta - alpha)
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w
        
    target = getTargetPosition('/Cylinder[0]')
    robot = getTargetPosition('/PioneerP3DX[0]')
    x_diff = target[0] - robot[0]
    y_diff = target[1] - robot[1]
    orientation = getTargetOrientation('/PioneerP3DX[0]')
        
    theta_goal = atan2(y_diff, x_diff)
    rho, v, w = calc_control_command(self, x_diff, y_diff, orientation[-1], theta_goal)
    
    self.vLeft,self.vRight = uniCyclevel(v,w)
    if rho<0.15:
        self.vLeft,self.vRight = 0,0
    #print(f'vLeft ={vLeft},vRight = {vRight}')
    sim.setJointTargetVelocity(self.motorLeft,self.vLeft)
    sim.setJointTargetVelocity(self.motorRight,self.vRight)
        
        
    
