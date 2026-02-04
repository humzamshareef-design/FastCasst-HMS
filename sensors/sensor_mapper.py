# ================================================================
# Universal Sensor Mapper for FASTCASST SIL
# Maps vehicle truth states to sensor stubs
# Works for any vehicle exposing: x, y, u, psi
# ================================================================

class SensorMapper:

    def __init__(self):
        pass

    def update(self, vehicle, gps, imu, baro):

        # --- GPS ---
        gps.latitude  = vehicle.x
        gps.longitude = vehicle.y
        gps.speed     = vehicle.u

        # --- IMU (yaw only for planar motion) ---
        imu.rpy[2] = vehicle.psi
        imu.g[0] = 0.0
        imu.g[1] = 0.0
        imu.g[2] = 0.0

        # --- Barometer (flat Earth, constant altitude) ---
        baro.ALT = 0.0
