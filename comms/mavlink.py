from pymavlink import mavutil
import math

VEHICLE_MAV_TYPE = {
    "boat": mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
    "car": mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
    "airplane": mavutil.mavlink.MAV_TYPE_FIXED_WING
     }

class MavlinkBridge:
    def __init__(self, vehicle_type="boat", port=5760):
        self.mav_type = VEHICLE_MAV_TYPE.get( vehicle_type, mavutil.mavlink.MAV_TYPE_GENERIC)

        print("Starting MAVLink TCP server on port", port)
        self.master = mavutil.mavlink_connection(f"tcpin:0.0.0.0:{port}", source_system=1, source_component=1)

        print("Waiting for Mission Planner connection...")
        self.master.wait_heartbeat()
        print("Mission Planner connected!")

        self.last_hb = 0.0
        self.home_sent = False

    def send_state(self, vehicle, gps, imu, t):

        # ---------------- HEARTBEAT (2 Hz) ----------------
        if (t - self.last_hb) > 0.5:
            self.master.mav.heartbeat_send(
                self.mav_type,
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                mavutil.mavlink.MAV_MODE_MANUAL_ARMED,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE
            )

        # ---------------- FRAME FIX ----------------
        # FASTCASST: x forward, y right
        # MAVLink:   North forward, East right
        north = vehicle.x
        east  = vehicle.y

        lat, lon = self.meters_to_gps(north, east)

        # ---------------- GPS (fix + auto-home) ----------------
        self.master.mav.gps_raw_int_send(
            int(t * 1e6),             # time_usec
            3,                        # fix_type (3D)
            int(lat * 1e7),           # lat
            int(lon * 1e7),           # lon
            0,                        # alt (mm)
            255, 255,                 # eph, epv
            int(vehicle.u * 100),     # vel (cm/s)
            0,                        # cog
            10                        # satellites
        )

        # ---------------- SET HOME (COMMAND_LONG) ----------------
        if not self.home_sent:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,
                1,                    # use current location
                0, 0, 0,
                lat,                  # latitude (deg)
                lon,                  # longitude (deg)
                0                     # altitude
            )
            self.home_sent = True

        # ---------------- ATTITUDE ----------------
        yaw_ned = -vehicle.psi       # NED uses clockwise positive
        self.master.mav.attitude_send(
            int(t * 1000),
            0.0,
            0.0,
            yaw_ned,
            0.0,
            0.0,
            0.0
        )

        # ---------------- GLOBAL POSITION ----------------
        self.master.mav.global_position_int_send(
            int(t * 1000),
            int(lat * 1e7),
            int(lon * 1e7),
            0,
            0,
            int(vehicle.u * 100),
            0,
            0,
            int(math.degrees(vehicle.psi) * 100)
        )

        # ---------------- HUD ----------------
        self.master.mav.vfr_hud_send(
            vehicle.u,
            vehicle.u,
            int(math.degrees(vehicle.psi)) % 360,
            int(vehicle.throttle * 100),
            0.0,
            0.0
        )

        # ---------------- PARAM HANDSHAKE ----------------
        msg = self.master.recv_match(type='PARAM_REQUEST_LIST', blocking=False)
        if msg:
            self.master.mav.param_value_send(
                b'NONE',
                0.0,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                0,
                0
            )

    def meters_to_gps(self, north, east):
        # Gulf Coast of Alabama
        lat0 = 28.4275
        lon0 = -87.6879
        R = 6378137.0

        dlat = north / R
        dlon = east / (R * math.cos(math.radians(lat0)))

        lat = lat0 + math.degrees(dlat)
        lon = lon0 + math.degrees(dlon)

        return lat, lon
