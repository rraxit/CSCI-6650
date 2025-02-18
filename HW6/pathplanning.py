#python
import numpy as np
import math
def sysCall_init():
    sim = require('sim')
    
    self.pathFound = False
    self.objectToFollowPath = sim.getObject('/Cylinder[0]')
    self.objectToFollowPath1 = sim.getObject('/Cylinder[1]')
    self.velocity = 0.1 # m/s
    self.velocity1 = 0.1 # m/s
    self.posAlongPath = 0
    self.posAlongPath1 = 0
    self.previousSimulationTime = 0
    sim.setStepping(True)
    self.pathComplete = False
    self.pathComplete1 = False
    

def sysCall_actuation():
    pathHandle = sim.getInt32Signal('path')
    if pathHandle is not None and not self.pathFound:
        print(f'pathHandle is {pathHandle}')
        self.pathFound = True
        pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(pathHandle, 'PATH'))
        self.path = pathHandle
        m = np.array(pathData).reshape(len(pathData) // 7, 7)
        n=np.flip(m, axis=0)
        self.pathPositions = m[:, :3].flatten().tolist()
        self.pathQuaternions = m[:, 3:].flatten().tolist()
        self.pathLengths, self.totalLength = sim.getPathLengths(self.pathPositions, 3)
        
        self.pathPositions1 = n[:, :3].flatten().tolist()
        self.pathQuaternions1 = n[:, 3:].flatten().tolist()
        self.pathLengths1, self.totalLength1 = sim.getPathLengths(self.pathPositions1, 3)
    elif self.pathFound:
        if not self.pathComplete or not self.pathComplete1:
            t = sim.getSimulationTime()
            self.posAlongPath += self.velocity * (t - self.previousSimulationTime)
            self.posAlongPath1 += self.velocity1 * (t - self.previousSimulationTime)
            
            if self.posAlongPath >= self.totalLength:
                self.posAlongPath = self.totalLength  # Ensure it does not exceed path length
                self.pathComplete = True

            if self.posAlongPath1 >= self.totalLength1:
                self.posAlongPath1 = self.totalLength1  # Ensure it does not exceed path length
                self.pathComplete1 = True

            #self.posAlongPath %= self.totalLength
            #self.posAlongPath1 %= self.totalLength1
            pos = sim.getPathInterpolatedConfig(self.pathPositions, self.pathLengths, self.posAlongPath)
            quat = sim.getPathInterpolatedConfig(self.pathQuaternions, self.pathLengths,
                self.posAlongPath, None, [2, 2, 2, 2])
            
            pos1 = sim.getPathInterpolatedConfig(self.pathPositions1, self.pathLengths1, self.posAlongPath1)
            quat1 = sim.getPathInterpolatedConfig(self.pathQuaternions1, self.pathLengths1,
                self.posAlongPath1, None, [2, 2, 2, 2])
            #print(pos)
            sim.setObjectPosition(self.objectToFollowPath, pos, self.path)
            sim.setObjectQuaternion(self.objectToFollowPath, quat, self.path)
            sim.setObjectPosition(self.objectToFollowPath1, pos1, self.path)
            sim.setObjectQuaternion(self.objectToFollowPath, quat, self.path)
            self.previousSimulationTime = t
            if self.pathComplete and self.pathComplete1:
                sim.stopSimulation()  # Stop the simulation after one complete round
            

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass
def getTargetPosition(name):
    objHandle = sim.getObject(name)
    pos = sim.getObjectPosition(objHandle, -1)
    return pos
    
def linear_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
# See the user manual or the available code snippets for additional callback functions and details