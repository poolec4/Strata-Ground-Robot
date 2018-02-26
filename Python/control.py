import numpy as np
import math
import time

def sind(angle):
    return math.sin(angle*math.pi/180)

class controller:
    def __init__(self, init_angle=135.0, version='v1.0', bounds=(100, 160)):
        self.version = version
        self.bounds = bounds
        # Controller Weights
        self.A = 800*np.ones(6)
        self.B = 800*np.ones(6)
        self.C = 0*np.ones(6)
        self.D = 50*np.asarray([1, 1, 0, 0, 0, 0]) # Modify to Improve?
        self.E = 50*np.asarray([0, 0, 1, 1, 1, 1]) # Modify to Improve?
        self.F = 20*np.ones(6)
        self.G = np.asarray([0.0008])
        # Error Data
        self.error = np.zeros(6)
        # Initial Angles
        self.theta = np.asarray([init_angle, -init_angle, init_angle, -init_angle, init_angle, -init_angle]).reshape(6, 1)
        self.d_theta = np.zeros(6)
        # Time
        self.t_old = time.time()

    def step(self, angles, translation, t):
        dt = np.asarray([t-self.t_old])
        #print(dt)
        if self.version == 'v1.0':
            self = self.PI_control(angles, translation, dt)
        else:
            raise ValueError('Invalid Controller Version Selected')
        self = self.bound()
        self.t_old = t
        return self
    
    def bound(self):
        for i in range(0, 6):
            if abs(self.theta[i]) < self.bounds[0]:
                if self.theta[i] > 0:
                    self.theta[i] = self.bounds[0]
                else:
                    self.theta[i] = -self.bounds[0]
                    
            elif abs(self.theta[i]) > self.bounds[1]:
                if self.theta[i] > 0:
                    self.theta[i] = self.bounds[1]
                else:
                    self.theta[i] = -self.bounds[1]
                
        return self


    def PI_control(self, angles, translation, dt): # Angles Must be in Radians
        # Proportional Control
        self.d_theta = [[self.A[0]*sind(angles[0])+self.B[0]*sind(angles[1])-self.C[0]*sind(angles[2])+self.D[0]*translation[0]+self.E[0]*translation[1]-self.F[0]*translation[2]],
                [self.A[1]*sind(angles[0])-self.B[1]*sind(angles[1])-self.C[1]*sind(angles[2])+self.D[1]*translation[0]+self.E[1]*translation[1]+self.F[1]*translation[2]],
                [self.A[2]*sind(angles[0])-self.B[2]*sind(angles[1])-self.C[2]*sind(angles[2])+self.D[2]*translation[0]+self.E[2]*translation[1]-self.F[2]*translation[2]],
                [-self.A[3]*sind(angles[0])+self.B[3]*sind(angles[1])-self.C[3]*sind(angles[2])+self.D[3]*translation[0]+self.E[3]*translation[1]+self.F[3]*translation[2]],
                [-self.A[4]*sind(angles[0])-self.B[4]*sind(angles[1])-self.C[4]*sind(angles[2])+self.D[4]*translation[0]-self.E[4]*translation[1]-self.F[4]*translation[2]],
                [self.A[5]*sind(angles[0])+self.B[5]*sind(angles[1])-self.C[5]*sind(angles[1])+self.D[5]*translation[0]-self.E[5]*translation[1]+self.F[5]*translation[2]]]
        # Integral Control
        self.error = np.asarray([sind(angles[0]), sind(angles[1]), sind(angles[2]), translation[0], translation[1], translation[2]]) + 0.9*self.error
        self.d_theta += self.G*[[self.A[0]*self.error[0]+self.B[0]*self.error[1]-self.C[0]*self.error[2]+self.D[0]*self.error[3]+self.E[0]*self.error[4]-self.F[0]*self.error[5]],
            [self.A[1]*self.error[0]-self.B[1]*self.error[1]-self.C[1]*self.error[2]+self.D[1]*self.error[3]+self.E[1]*self.error[4]+self.F[1]*self.error[5]],
            [self.A[2]*self.error[0]-self.B[2]*self.error[1]-self.C[2]*self.error[2]+self.D[2]*self.error[3]+self.E[2]*self.error[4]-self.F[2]*self.error[5]],
            [-self.A[3]*self.error[0]+self.B[3]*self.error[1]-self.C[3]*self.error[2]+self.D[3]*self.error[3]+self.E[3]*self.error[4]+self.F[3]*self.error[5]],
            [-self.A[4]*self.error[0]-self.B[4]*self.error[1]-self.C[4]*self.error[2]+self.D[4]*self.error[3]-self.E[4]*self.error[4]-self.F[4]*self.error[5]],
            [self.A[5]*self.error[0]+self.B[5]*self.error[1]-self.C[5]*self.error[2]+self.D[5]*self.error[3]-self.E[5]*self.error[4]+self.F[5]*self.error[5]]]
        #print(self.d_theta*dt)
        #print(self.theta)
        self.theta += self.d_theta*dt
        return self
