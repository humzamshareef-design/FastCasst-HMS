import numpy as np

class Vehicle:

    def __init__(self):
        # Control effectiveness gains (simple, tunable)
        self.K_throttle = 10.0   # m/s^2 per throttle
        self.K_pitch = 0.6       # rad/s per elevator
        self.K_yaw = 0.5         # rad/s per rudder

        # States
        self.x = 0.0
        self.y = 0.0
        self.h = 100.0           # start above ground
        self.u = 20.0            # initial airspeed
        self.psi = 0.0
        self.gamma = 0.0

        # Inputs
        self.throttle = 0.0
        self.elevator = 0.0
        self.rudder = 0.0

    def apply_controls(self, controls, rc):
        """
        FASTCASST V_airplane convention:
            controls[0] = throttle
            controls[1] = aileron (ignored in kinematic model)
            controls[2] = elevator
            controls[3] = rudder
        """
        self.throttle = float(controls[0])
        self.elevator = float(controls[2])
        self.rudder   = float(controls[3])

    def step(self, dt):
        # Airspeed dynamics
        du = self.K_throttle * self.throttle
        self.u += du * dt

        # Flight path angle (pitch) dynamics
        gamma_dot = self.K_pitch * self.elevator
        self.gamma += gamma_dot * dt

        # Heading (yaw) dynamics
        psi_dot = self.K_yaw * self.rudder
        self.psi += psi_dot * dt

        # Kinematic position update
        self.x += self.u * np.cos(self.gamma) * np.cos(self.psi) * dt
        self.y += self.u * np.cos(self.gamma) * np.sin(self.psi) * dt
        self.h += self.u * np.sin(self.gamma) * dt
