import pandas as pd
import matplotlib.pyplot as plt

CSV_FILE = "/Users/michaelb/Downloads/sensor_data_2_09.csv"
SAMPLE_RATE = 1000
RAW_SENSOR_CSV_COLUMNS = [
    "Front Weight",
    "Back Weight",
    "Raw Left Surge",
    "Raw Right Surge",
    "Left IMU Acc X",
    "Left IMU Acc Y",
    "Left IMU Acc Z",
    "Left IMU Gyro X",
    "Left IMU Gyro Y",
    "Left IMU Gyro Z",
    "Left IMU Mag X",
    "Left IMU Mag Y",
    "Left IMU Mag Z",
    "Right IMU Acc X",
    "Right IMU Acc Y",
    "Right IMU Acc Z",
    "Right IMU Gyro X",
    "Right IMU Gyro Y",
    "Right IMU Gyro Z",
    "Right IMU Mag X",
    "Right IMU Mag Y",
    "Right IMU Mag Z",
]

# Load CSV
df = pd.read_csv(CSV_FILE, header=None)

df = df.iloc[::SAMPLE_RATE]

# First column is time
time = df.iloc[:, 0]

# Remaining 22 columns are measurements
values = df.iloc[:, 1:23]

# Plot
plt.figure(figsize=(14, 8))

for i in range(values.shape[1]):
    plt.plot(time, values.iloc[:, i], label=RAW_SENSOR_CSV_COLUMNS[i])

plt.xlabel("Time")
plt.ylabel("Value")
plt.title("Sensor Data Over Time")

plt.legend()
plt.grid(True)
plt.tight_layout()

plt.show()
