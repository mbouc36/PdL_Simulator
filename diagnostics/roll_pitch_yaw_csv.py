import serial
import csv
import time

SERIAL_PORT = "/dev/cu.wchusbserial1330"  
BAUD_RATE = 115200 
TIMEOUT = 0.1

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)  # Open serial port
    ser.reset_input_buffer()  # Clear input buffer to remove stale data
except serial.SerialException as e:
    print(f"Failed to open port at {SERIAL_PORT}")
    exit(1)


CSV_FILE = "imu_data.csv"

# Create CSV file and write header
with open(CSV_FILE, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["timestamp", "roll", "pitch", "yaw"])

print(f"Saving IMU data to {CSV_FILE}...")

try:
    while True:
        if ser.in_waiting <= 0:
            continue

        try:
            line = ser.readline().decode("utf-8").strip()
            values = line.split(",")
            
            if len(values) != 7:
                print("Incorrect number of values being printed") 

            timestamp, roll, pitch, yaw = map(float, values)


            # Append data to CSV
            with open(CSV_FILE, mode="a", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, roll, pitch, yaw])

            print(f"{timestamp:.2f}, {roll:.2f}, {pitch:.2f}, {yaw:.2f}")
        except Exception:
            print("An error occured when parsing the serial port")

        #time.sleep(0.1)  # 10 Hz logging rate

except KeyboardInterrupt:
    print("\nLogging stopped.")