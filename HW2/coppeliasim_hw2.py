#python
import numpy as np
import random

def getObjectState(target):
    p = sim.getObjectPosition(target, -1)
    return np.array([p[0], p[1]])

def is_point_inside_triangle(A, B, C, P):
    # Create matrices
    M = np.array([[B[0] - A[0], C[0] - A[0]], [B[1] - A[1], C[1] - A[1]]])
    M_AP = np.array([[B[0] - A[0], P[0] - A[0]], [B[1] - A[1], P[1] - A[1]]])

    # Calculate determinants
    det_M = np.linalg.det(M)
    det_M_AP = np.linalg.det(M_AP)

    # Check if the point is inside the triangle
    return 0 < det_M * det_M_AP and det_M * det_M_AP <= det_M * det_M
    
def sysCall_init():
    sim = require('sim')

    # Put some initialization code here
    # sim.setStepping(True) # enabling stepping mode
    #
    # Instead of using globals, you can do e.g.:
    # self.myVariable = 21000000

def sysCall_thread():
    # Put your main code here, e.g.:
    obs = (([-2.5, 2.5],[0, 0.8333],[2.5, -2.5],[-2.5, 0.8333]))
    ws = (([5, 5],[5, -5],[-5, -5],[-5, 5]))
    U = np.array(([0, 0.1],[0, -0.1],[0.1, 0],[-0.1, 0]))
    target = sim.getObject("/target")
    while not sim.getSimulationStopping():
         x = getObjectState(target)
         u = random.choice(U)
         nx= x+u
         print(nx)
         Point= [nx[0], nx[1], 0.5]
         print(is_point_inside_triangle(obs[0],obs[1],obs[2],nx))
         #print(is_point_inside_triangle(workspace[0], workspace[1], workspace[2], nx))
         #print(is_point_inside_triangle(workspace[0], workspace[3], workspace[2], nx))
         
         if is_point_inside_triangle(ws[0], ws[1], ws[2], nx)== True and is_point_inside_triangle(ws[0], ws[3], ws[2], nx)== True:
             if is_point_inside_triangle(obs[0],obs[1],obs[2],nx)== False and is_point_inside_triangle(obs[0],obs[3],obs[2],nx)== False:
                 sim.setObjectPosition(target, -1, Point)
                 sim.step() # resume in next simulation step
                 
            
            
    pass

# See the user manual or the available code snippets for additional callback functions and details

