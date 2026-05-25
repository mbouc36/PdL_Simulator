import time
import math
import numpy as np
import serial
from ahrs.filters import Madgwick
from ahrs.common.quaternion import Quaternion




# Offsets and Scale constants
accOffset = {}
accScale = {}
magOffset = {}
magScale = {}

accOffset["x"] = 823.8871291297346
accOffset["y"] = -682.270126084291
accOffset["z"] = 842.115949003557
accScale["x"] = 5.847155678330655e-05
accScale["y"] = 5.847155678330655e-05
accScale["z"] = 5.847155678330655e-05
gxOffset = -0.31884999999999997
gyOffset = 0.04880000000000001
gzOffset = -0.35644999999999993
magOffset["x"] = -4154.660891620942
magOffset["y"] = 2211.639199618942
magOffset["z"] = 777.9524384752815
magScale["x"] = 0.0002773468275482515
magScale["y"] = 0.000244972282258551
magScale["z"] = 0.00027722357289711766


SERIAL_PORT = "/dev/cu.wchusbserial1330"
BAUD_RATE = 115200
FREQUENCY = 200.0
K_P = 1.0
K_I = 0.01
GAUSS_TO_MILLI_TESLA_CONVERSION = 10

class BodyFrameFusedRotation:
    def __init__(self):
        self.filter = Madgwick(sample_period=1/FREQUENCY, gain=0.033)
        self.q_prev = np.array([1.0, 0.0, 0.0, 0.0])
        
        # This is your final, drift-corrected body Z-axis tracker
        self.total_body_z_angle = 0.0 
        
    def update(self, gyro, accel, mag):
        # 1. Update the global inertial orientation (Madgwick filter)
        q_curr = self.filter.updateMARG(self.q_prev, gyr=gyro, acc=accel, mag=mag)
        
        # 2. Calculate the inverse (conjugate) of the previous quaternion
        # For a normalized quaternion [w, x, y, z], the inverse is [w, -x, -y, -z]
        q_prev_inv = np.array([self.q_prev[0], -self.q_prev[1], -self.q_prev[2], -self.q_prev[3]])
        
        # 3. Calculate the Delta Quaternion in the body frame
        # We use the AHRS Quaternion class to easily multiply them
        q_prev_inv_obj = Quaternion(q_prev_inv)
        q_curr_obj = Quaternion(q_curr)
        delta_q_body = q_prev_inv_obj * q_curr_obj # q_prev^-1 * q_curr
        
        # delta_q_body is now [w, x, y, z]
        w, x, y, z = delta_q_body
        
        # 4. Extract just the rotation around the local Z-axis (in radians)
        delta_z_rad = 2.0 * np.arctan2(z, w)
        
        # 5. Convert to degrees and accumulate
        delta_z_deg = np.degrees(delta_z_rad)
        self.total_body_z_angle += delta_z_deg
        
        # Store current state for the next loop
        self.q_prev = q_curr
        
        return self.total_body_z_angle


# (Assume the BodyFrameFusedRotation class from the previous answer is defined here)

def read_raw_sensors(values):
    """
    PLACEHOLDER: Replace this with your specific I2C/SPI sensor reads.
    
    CRITICAL UNIT REQUIREMENTS FOR AHRS FILTERS:
    - Gyroscope: MUST be in radians per second (rad/s)
    - Accelerometer: Meters per second squared (m/s^2), or simply normalized
    - Magnetometer: Microteslas (uT), or simply normalized
    """

    ax, ay, az, gx, gy, gz, mx, my, mz = map(float, values)

    axCal = (ax - accOffset["x"]) * accScale["x"]
    ayCal = (ay - accOffset["y"]) * accScale["y"]
    azCal = (az - accOffset["z"]) * accScale["z"]

    gxCal = math.radians(gx - gxOffset)
    gyCal = math.radians(gy - gyOffset)
    gzCal = math.radians(gz - gzOffset)

    mxCal = (
        (mx - magOffset["x"]) * magScale["x"]
    ) / GAUSS_TO_MILLI_TESLA_CONVERSION
    myCal = (
        (my - magOffset["y"]) * magScale["y"]
    ) / GAUSS_TO_MILLI_TESLA_CONVERSION
    mzCal = (
        (mz - magOffset["z"]) * magScale["z"]
    ) / GAUSS_TO_MILLI_TESLA_CONVERSION

    gyro_data = np.array([gxCal, gyCal, gzCal])
    acc_data = np.array([axCal, ayCal, azCal])
    mag_data = np.array([mxCal, myCal, mzCal])
    
    return gyro_data, acc_data, mag_data

if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except Exception:
        print(f"Failed to connect to port: {SERIAL_PORT}")
        exit(1)

    # Initialize the tracker
    tracker = BodyFrameFusedRotation()
    
    # This must match the sample_period inside your Madgwick filter setup
    loop_dt = 1/FREQUENCY  # 200 Hz
        
    try:
        while True:

            line = ser.readline().decode("utf-8").strip()
            values = line.split(",")

            if len(values) != 9:
                continue

            
            # 1. Grab the fresh data from the IMU
            gyro, accel, mag = read_raw_sensors(values)
            
            # 2. Feed the vectors into the filter and get the delta-calculated Z angle
            current_z_angle = tracker.update(gyro, accel, mag)
            
            # 3. Print or use your perfectly drift-corrected body-frame rotation
            print(f"Drift-Corrected Body Z-Rotation: {current_z_angle:.2f}°")
            
                
    except KeyboardInterrupt:
        print("\nTracking stopped.")