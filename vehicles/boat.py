import numpy as np

class Vehicle:

    def __init__(self):
        # Physical parameters
        self.mass = 50.0
        self.drag = 8.0
        self.mdot_max = 6.0
        self.Vjet = 15.0
        self.K_jet = 0.8   # yaw authority

        # States
        self.x = 0.0
        self.y = 0.0
        self.u = 0.0
        self.psi = 0.0

        # Inputs
        self.throttle = 0.0
        self.steering = 0.0

    def apply_controls(self, controls, rc):
        # FASTCASST V_boat returns scalar throttle
        self.throttle = float(controls[0])
        self.steering = rc.yawrc

    def step(self, dt):
        # Surge
        mdot = self.throttle * self.mdot_max
        thrust = mdot * self.Vjet
        drag_force = self.drag * self.u * abs(self.u)

        du = (thrust - drag_force) / self.mass
        self.u += du * dt

        # Yaw (jet-based)
        psi_dot = self.K_jet * self.steering * abs(self.throttle)
        self.psi += psi_dot * dt

        # Planar kinematics
        self.x += self.u * np.cos(self.psi) * dt
        self.y += self.u * np.sin(self.psi) * dt
