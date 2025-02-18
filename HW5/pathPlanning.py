#python
import numpy as np
def sysCall_init():
    sim = require('sim')
    #ori_path = np.array([[0,0,0.1,0,0,0,1],[0,1,0.1,0,0,0,1]])
    #path = ori_path.flatten().tolist()
    #self.path = sim.createPath(path)
    self.pathFound = False
    self.objectToFollowPath = sim.getObject('/Cylinder')
    #self.pathPositions = ori_path[:, :3].flatten().tolist()
    #self.pathQuaternions = ori_path[:, 3:].flatten().tolist()
    
    #self.pathLengths, self.totalLength = sim.getPathLengths(self.pathPositions, 3)
    self.velocity = 0.1 # m/s
    self.posAlongPath = 0
    self.previousSimulationTime = 0
    sim.setStepping(True)
    
    

    # do some initialization here

def sysCall_actuation():
    # put your actuation code here
    
    pathHandle = sim.getInt32Signal('path')
    if pathHandle is not None and not self.pathFound:
        print(f'pathHandle is {pathHandle}')
        self.pathFound = True
        pathData = sim.unpackDoubleTable(sim.readCustomDataBlock(pathHandle, 'PATH'))
        self.path = pathHandle
        m = np.array(pathData).reshape(len(pathData) // 7, 7)
        self.pathPositions = m[:, :3].flatten().tolist()
        self.pathQuaternions = m[:, 3:].flatten().tolist()
        self.pathLengths, self.totalLength = sim.getPathLengths(self.pathPositions, 3)
    elif self.pathFound:
        t = sim.getSimulationTime()
        self.posAlongPath += self.velocity * (t - self.previousSimulationTime)
        self.posAlongPath %= self.totalLength
        pos = sim.getPathInterpolatedConfig(self.pathPositions, self.pathLengths, self.posAlongPath)
        quat = sim.getPathInterpolatedConfig(self.pathQuaternions, self.pathLengths,
            self.posAlongPath, None, [2, 2, 2, 2])
        #print(pos)
        sim.setObjectPosition(self.objectToFollowPath, pos, self.path)
        sim.setObjectQuaternion(self.objectToFollowPath, quat, self.path)
        self.previousSimulationTime = t
        #self.objectToFollowPath = sim.getObject('/Cylinder')

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    # do some clean-up here
    pass

# See the user manual or the available code snippets for additional callback functions and details