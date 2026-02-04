import numpy as np

class Vehicle:
    """
    Kinematic Bicycle Model
    States: x, y, u, psi
    """

    def __init__(self):
        # Vehicle parameters
        self.L = 2.5      # wheelbase (m)
        self.a_max = 3.0  # max accel (m/s^2)
        self.c_d = 0.3    # rolling + aero drag

        # States
        self.x = 0.0
        self.y = 0.0
        self.u = 0.0
        self.psi = 0.0

        # Inputs
        self.throttle = 0.0
        self.delta = 0.0

    def apply_controls(self, controls, rc):
        # FASTCASST V_car uses RC for throttle & steering
        self.throttle = rc.throttlerc
        self.delta    = rc.yawrc


    def step(self, dt):
        # --- Longitudinal dynamics ---
        # Engine minus drag
        du = self.a_max * self.throttle - self.c_d * self.u
        self.u += du * dt

        # --- Yaw kinematics (bicycle model) ---
        psi_dot = (self.u / self.L) * np.tan(self.delta)
        self.psi += psi_dot * dt

        # --- Planar kinematics ---
        self.x += self.u * np.cos(self.psi) * dt
        self.y += self.u * np.sin(self.psi) * dt
