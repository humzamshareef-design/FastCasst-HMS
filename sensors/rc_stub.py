class RCStub:

    def __init__(self):
        # RC channel values in [-1, 1]
        self.rollrc = 0.0
        self.pitchrc = 0.0
        self.yawrc = 0.0
        self.throttlerc = 0.0

        # Autopilot mode: 0=manual, 1=stabilized, 2=auto
        self.autopilot = 1

        # For array-style access (if needed)
        self.rcsignals = [0.0, 0.0, 0.0, 0.0]

    def set_roll(self, val):
        self.rollrc = val
        self.rcsignals[0] = val

    def set_pitch(self, val):
        self.pitchrc = val
        self.rcsignals[1] = val

    def set_yaw(self, val):
        self.yawrc = val
        self.rcsignals[2] = val

    def set_throttle(self, val):
        self.throttlerc = val
        self.rcsignals[3] = val

    # Backward-compatible names you used earlier
    def set_steering(self, val):
        self.set_yaw(val)

    def set_throttle_cmd(self, val):
        self.set_throttle(val)
