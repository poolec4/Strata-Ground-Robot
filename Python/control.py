import numpy as np
from math import sin

class controller:
    def __init__(self, init_angle=20, version='PI', bounds=(-45, 45)):
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
        if self.version == 'PI':
            self = self.PI_control(angles, translation, dt)

        self.t_old = t
        return self


    def PI_control(self angles, translation, dt): # Angles Must be in Radians
        # Proportional Control
        d_theta = [[A[1]*sin(angles[1])+B[1]*sin(angles[2])-C[1]*sin(angles[3])+D[1]*translation[1]+E[1]*translation[2]-F[1]*translation[3]]
                [A[2]*sin(angles[1])-B[2]*sin(angles[2])-C[2]*sin(angles[3])+D[2]*translation[1]+E[2]*translation[2]+F[2]*translation[3]]
                [A[3]*sin(angles[1])-B[3]*sin(angles[2])-C[3]*sin(angles[3])+D[3]*translation[1]+E[3]*translation[2]-F[3]*translation[3]]
                [-A[4]*sin(angles[1])+B[4]*sin(angles[2])-C[4]*sin(angles[3])+D[4]*translation[1]+E[4]*translation[2]+F[4]*translation[3]]
                [-A[5]*sin(angles[1])-B[5]*sin(angles[2])-C[5]*sin(angles[3])+D[5]*translation[1]-E[5]*translation[2]-F[5]*translation[3]]
                [A[6]*sin(angles[1])+B[6]*sin(angles[2])-C[6]*sin(angles[2])+D[6]*translation[1]-E[6]*translation[2]+F[6]*translation[3]]]
        # Integral Control
        self.error = np.asarray([sin(angles[1]), sin(angles[2]), sin(angles[3]), translation[1], translation[2], translation[3]]) + 0.9*self.error
        d_theta += G*[[A[1]*self.error[1]+B[1]*self.error[2]-C[1]*self.error[3]+D[1]*self.error[4]+E[1]*self.error[5]-F[1]*self.error[6]]
            [A[2]*self.error[1]-B[2]*self.error[2]-C[2]*self.error[3]+D[2]*self.error[4]+E[2]*self.error[5]+F[2]*self.error[6]]
            [A[3]*self.error[1]-B[3]*self.error[2]-C[3]*self.error[3]+D[3]*self.error[4]+E[3]*self.error[5]-F[3]*self.error[6]]
            [-A[4]*self.error[1]+B[4]*self.error[2]-C[4]*self.error[3]+D[4]*self.error[4]+E[4]*self.error[5]+F[4]*self.error[6]]
            [-A[5]*self.error[1]-B[5]*self.error[2]-C[5]*self.error[3]+D[5]*self.error[4]-E[5]*self.error[5]-F[5]*self.error[6]]
            [A[6]*self.error[1]+B[6]*self.error[2]-C[6]*self.error[3]+D[6]*self.error[4]-E[6]*self.error[5]+F[6]*self.error[6]]]

        self.theta += d_theta*dt
        return self
