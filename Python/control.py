import numpy as np
from math import sin

class controller:
    def __init__(self, init_angle=20, version='v1.0', bounds=(-45, 45)):
        self.version = version
        self.bounds = bounds
        # Controller Weights
        self.A = 250*np.ones(6)
        self.B = 250*np.ones(6)
        self.C = 100*np.ones(6)
        self.D = 500*np.asarray([1, 1, 0, 0, 0, 0]) # Modify to Improve?
        self.E = 500*np.asarray([0, 0, 1, 1, 1, 1]) # Modify to Improve?
        self.F = 200*np.ones(6)
        self.G = 0.25
        # Error Data
        self.error = np.zeros(6)
        # Initial Angles
        self.theta = np.asarray([init_angle, -init_angle, init_angle, -init_angle, init_angle, -init_angle])
        # Time
        self.t_old = 0

    def step(self, angles, translation, t):
        dt = t-self.t_old
        if self.version == 'v1.0':
            self = self.PI_control(angles, translation, dt)
        else:
            raise ValueError('Invalid Controller Version Selected')
        self.t_old = t
        return self


    def PI_control(self, angles, translation, dt): # Angles Must be in Radians
        # Proportional Control
        d_theta = [[self.A[1]*sin(angles[1])+self.B[1]*sin(angles[2])-self.C[1]*sin(angles[3])+self.D[1]*translation[1]+self.E[1]*translation[2]-self.F[1]*translation[3]]
                [self.A[2]*sin(angles[1])-self.B[2]*sin(angles[2])-self.C[2]*sin(angles[3])+self.D[2]*translation[1]+self.E[2]*translation[2]+self.F[2]*translation[3]]
                [self.A[3]*sin(angles[1])-self.B[3]*sin(angles[2])-self.C[3]*sin(angles[3])+self.D[3]*translation[1]+self.E[3]*translation[2]-self.F[3]*translation[3]]
                [-self.A[4]*sin(angles[1])+self.B[4]*sin(angles[2])-self.C[4]*sin(angles[3])+self.D[4]*translation[1]+self.E[4]*translation[2]+self.F[4]*translation[3]]
                [-self.A[5]*sin(angles[1])-self.B[5]*sin(angles[2])-self.C[5]*sin(angles[3])+self.D[5]*translation[1]-self.E[5]*translation[2]-self.F[5]*translation[3]]
                [self.A[6]*sin(angles[1])+self.B[6]*sin(angles[2])-self.C[6]*sin(angles[2])+self.D[6]*translation[1]-self.E[6]*translation[2]+self.F[6]*translation[3]]]
        # Integral Control
        self.error = np.asarray([sin(angles[1]), sin(angles[2]), sin(angles[3]), translation[1], translation[2], translation[3]]) + 0.9*self.error
        d_theta += self.G*[[self.A[1]*self.error[1]+self.B[1]*self.error[2]-self.C[1]*self.error[3]+self.D[1]*self.error[4]+self.E[1]*self.error[5]-self.F[1]*self.error[6]]
            [self.A[2]*self.error[1]-self.B[2]*self.error[2]-self.C[2]*self.error[3]+self.D[2]*self.error[4]+self.E[2]*self.error[5]+self.F[2]*self.error[6]]
            [self.A[3]*self.error[1]-self.B[3]*self.error[2]-self.C[3]*self.error[3]+self.D[3]*self.error[4]+self.E[3]*self.error[5]-self.F[3]*self.error[6]]
            [-self.A[4]*self.error[1]+self.B[4]*self.error[2]-self.C[4]*self.error[3]+self.D[4]*self.error[4]+self.E[4]*self.error[5]+self.F[4]*self.error[6]]
            [-self.A[5]*self.error[1]-self.B[5]*self.error[2]-self.C[5]*self.error[3]+self.D[5]*self.error[4]-self.E[5]*self.error[5]-self.F[5]*self.error[6]]
            [self.A[6]*self.error[1]+self.B[6]*self.error[2]-self.C[6]*self.error[3]+self.D[6]*self.error[4]-self.E[6]*self.error[5]+self.F[6]*self.error[6]]]

        self.theta += d_theta*dt
        return self
