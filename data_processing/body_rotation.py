import serial
import numpy as np
from ahrs.filters import Madgwick
from ahrs.common.quaternion import Quaternion
from imu_helper_functions import get_imu_data, get_calibration_data



SERIAL_PORT = "/dev/cu.wchusbserial1330"
BAUD_RATE = 115200
FREQUENCY = 200.0
GAIN = 0.033

class IsolatedBodyAxisTracker:
    def __init__(self):
        self.filter = Madgwick(sample_period=1/FREQUENCY, gain=GAIN)
        self.q_prev = np.array([1.0, 0.0, 0.0, 0.0])
        
        # Three completely independent accumulated body-frame angles
        self.body_x_deg = 0.0
        self.body_y_deg = 0.0
        self.body_z_deg = 0.0
        

    def update(self, gyro, accel, mag):
        # 1. Get the global, drift-corrected quaternion from Madgwick
        q_curr = self.filter.updateMARG(self.q_prev, gyr=gyro, acc=accel, mag=mag)
        
        # 2. Isolate the movement strictly to the local body frame
        q_prev_inv = np.array([self.q_prev[0], -self.q_prev[1], -self.q_prev[2], -self.q_prev[3]])
        delta_q_body = Quaternion(q_prev_inv) * (Quaternion(q_curr))
        
        w, x, y, z = delta_q_body
        
        # 3. Extract independent local rotations (Infinitesimal approximation)
        # Because the time step is 0.01s, these are perfectly decoupled.
        delta_x = 2.0 * np.arctan2(x, w)
        delta_y = 2.0 * np.arctan2(y, w)
        delta_z = 2.0 * np.arctan2(z, w)
        
        # 4. Accumulate into your independent registers
        self.body_x_deg += np.degrees(delta_x)
        self.body_y_deg += np.degrees(delta_y)
        self.body_z_deg += np.degrees(delta_z)
        
        # Save state for the next frame
        self.q_prev = q_curr
        
        return self.body_x_deg, self.body_y_deg, self.body_z_deg
    

def get_x_y_z_angles():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except Exception:
        print(f"Failed to connect to port: {SERIAL_PORT}")
        exit(1)

    # Initialize the tracker
    tracker = IsolatedBodyAxisTracker()

    # Fetch calibration data from json 
    cal_data = get_calibration_data()
        
    try:
        while True:
            try:
                line = ser.readline().decode("utf-8").strip()
            except Exception as e:
                print(e)
                continue
            
            values = line.split(",")

            if len(values) != 9:
                continue

            # 1. Grab the fresh data from the IMU
            gyro, accel, mag = get_imu_data(values, cal_data)
            
            # 2. Feed the vectors into the filter and get the delta-calculated Z angle
            angles = tracker.update(gyro, accel, mag)
            
            # 3. Print or use your perfectly drift-corrected body-frame rotation
            # self.body_x_deg, self.body_y_deg, self.body_z_deg
            print(f"{angles[0].item():.4f}, {angles[1].item():.4f}, {angles[2].item():.4f}")
                   
    except KeyboardInterrupt:
        print("\nTracking stopped.")



if __name__ == "__main__":
    get_x_y_z_angles()
