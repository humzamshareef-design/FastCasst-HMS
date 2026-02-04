# ================================================================
# FASTCASST SIL Universal Simulator V2
# Updated for All Vehicles
# Author: Hamza Muhammad Shareef
# Advisor: Dr. Carlos Montalvo
# Created: Fall 2026
# FAST Laboratory - Facility for Aerial Systems & Technology
# University of South Alabama
# ================================================================

# ------------------------------------------------
# 1. Importing Libraries
# ------------------------------------------------
import sys
import os
import time
import inspect
import subprocess
import numpy as np


# ------------------------------------------------
# 2. Project Path Setup
# ------------------------------------------------
PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, PROJECT_ROOT)

# ------------------------------------------------
# 3. FASTCASST Log Directory 
# ------------------------------------------------
if len(sys.argv) == 1:
    sys.argv.append("logs/")

# ------------------------------------------------
# 3. FASTCASST Windows Utility Patch 
# ------------------------------------------------
sys.path.append(os.path.join(PROJECT_ROOT, "libraries", "Util"))
import util

def _isSIL_windows():
    return True

util.isSIL = _isSIL_windows

# ------------------------------------------------
# 4. Vehicle Selection
# ------------------------------------------------
VEHICLE = input("Select vehicle (boat / car / airplane): ")

# ------------------------------------------------
# 5. FASTCASST Controller
# ------------------------------------------------
sys.path.append(os.path.join(PROJECT_ROOT, "libraries", "V_" + VEHICLE))
import controller
autopilot = controller.CONTROLLER()

# ------------------------------------------------
# 6. Sensor-Adaptive Interface
# ------------------------------------------------
def call_controller_loop(ctrl, **kwargs):
    sig = inspect.signature(ctrl.loop)
    valid_args = {k: v for k, v in kwargs.items() if k in sig.parameters}
    out = ctrl.loop(**valid_args)
    controls, defaults, *extras = out
    return controls, defaults, extras

# ------------------------------------------------
# 7. Virtual Vehicle
# ------------------------------------------------
vehicle_module = __import__("vehicles." + VEHICLE, fromlist=["Vehicle"])
VehicleClass = vehicle_module.Vehicle
VirtualVehicle = VehicleClass()

# ------------------------------------------------
# 8. Sensor & RC Stubs and Sensor Mapper
# ------------------------------------------------
from sensors.rc_stub import RCStub
from sensors.sensor_stubs import GPSStub, IMUStub, BaroStub
from sensors.sensor_mapper import SensorMapper

rc = RCStub()
gps = GPSStub()
imu = IMUStub()
baro = BaroStub()
sensor_mapper = SensorMapper()

# ------------------------------------------------
# 9. Autopilot Mode
# ------------------------------------------------
rc.autopilot = 1 #Stabilized Mode

# ------------------------------------------------
# 10. FASTCASST Datalogger
# ------------------------------------------------
sys.path.append(os.path.join(PROJECT_ROOT, "libraries", "Datalogger"))
from datalogger import Datalogger

NUMOUTPUTS = 8 
logger = Datalogger(NUMOUTPUTS)

# ------------------------------------------------
# 11. Simulation Settings
# ------------------------------------------------
dt = 0.02
TEND = 20.0

RunTime = 0.0
StartTime = time.time()

#-------------------------------------------------

print("FASTCASST SIL Simulation")
print("Vehicle :", VEHICLE)

# ----------------------------------------------------------------
# 12. Main FastCasst SIL Loop
# ----------------------------------------------------------------
while RunTime < TEND:

    RunTime = time.time() - StartTime

    # --- Pilot Commands ---
    rc.set_throttle(0.8)
    rc.set_steering(0.0)

    # --- FASTCASST Autopilot ---
    controls, defaults, extras = call_controller_loop(
        autopilot,
        RunTime=RunTime,
        rcin=rc,
        gps=gps,
        gps_llh=gps,
        rpy=imu.rpy,
        g=imu.g,
        baro=baro
    )

    # Normalize scalar/vector controller outputs
    controls = np.atleast_1d(controls)

    # --- Vehicle Dynamics Propagation ---
    VirtualVehicle.apply_controls(controls, rc)
    VirtualVehicle.step(dt)

    # --- Sensor Update ---
    sensor_mapper.update(VirtualVehicle, gps, imu, baro)

    # --- FASTCASST Native Logging ---
    logger.outdata[0] = RunTime
    logger.outdata[1] = VirtualVehicle.x
    logger.outdata[2] = VirtualVehicle.y
    logger.outdata[3] = VirtualVehicle.u
    logger.outdata[4] = VirtualVehicle.psi
    logger.outdata[5] = rc.throttlerc
    logger.outdata[6] = rc.rollrc
    logger.outdata[7] = rc.autopilot
    logger.println()

    # --- Console Output ---
    print(
        f"t={RunTime:6.2f}  "
        f"x={VirtualVehicle.x:7.2f}  "
        f"y={VirtualVehicle.y:7.2f}  "
        f"u={VirtualVehicle.u:6.2f}  "
        f"psi={np.degrees(VirtualVehicle.psi):6.2f} deg"
    )

    time.sleep(dt)

print("Simulation Complete")

plot_script = os.path.join(PROJECT_ROOT, "plots", "plots.py")
subprocess.run([sys.executable, plot_script])
