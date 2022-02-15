import time
import numpy as np
import math as m
  
def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

class prop_filter:
        
    def __init__(self):
        self.magVect = np.array([1,0,0])
        self.gravVect = np.array([0,1,0])
    def update(self,accel,gyro,mag,timeStep):
        gyroScaled = gyro*timeStep
        rotation = (Rz(gyro[2]) * Ry(gyro[1]) * Rx(gyro[0])).dot(self.pose.T)
        self.grav = rotation.T.dot(self.grav.T)
        self.heading = rotation.dot(self.pose.T)
        self.grav = 0.9*self.grav + 0.1*accel/np.linalg.norm(accel)
        self