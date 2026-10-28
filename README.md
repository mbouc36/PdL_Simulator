# PdL_Simulator

## Install Requirements
`pip install pip-tools`

`pip install -r requirements.txt`

## Setup Config
Tip: To find serial ports on Unix based OS run: `ls /dev/cu.*`

run `python update_config.py`

## Usage 
1. Ensure all sensors are on and reading, check using **Arduino IDE**
2. Choose a path to create a name to key file, example: ~/Documents/name_to_key.csv
3. Run  `.\venv\Scripts\python.exe .\GUI\main.py -f {name_to_key_file}`
4. Output will be located in `output_data` under the respective key name which can be found in the name to key file

## Calibration

### IMU Calibration

1. Ensure you only have one IMU connected to the system. If a second IMU is connected remove it as seen in the image below:
<p align="center">
  <img width="500" alt="IMG_6090" src="https://github.com/user-attachments/assets/ec500283-3c7e-4d42-b04f-3b0fb98e905d" />
</p> 

2. Open the **Arduino IDE** and upload `calibration/imu_calibration/imu_calibration.ino`
<p align="center">
  <img width="500" alt="image" src="https://github.com/user-attachments/assets/f131921b-795a-4bbf-ac37-d1a1bba3f188" />
</p>

3. From the terminal  ensure the python virtual environment is active by running `.\venv\Scripts\activate` in `C:\Git\PdL_Simulator`. Then run `.\venv\Scripts\python.exe .\calibration\imu_calibration\imu_calibration.py`. The script will prompt the user for a name, usally 'left' or 'right' is used based on which hand the tool would be held with during training. Type the appropriate name, then press enter.
<p align="center">
  <img width="500" alt="image" src="https://github.com/user-attachments/assets/2859f4a3-2b1b-47d1-9010-e43d2f29fc83" />
</p>

4. The GUI should open. Two tabs can be seen, the **3D Magnetic Field Vector and Attitude** tab shows the orientation of the IMU in a 3D space, this is good for verification after calibration using this script. The **9-Axis IMU Visualization** will be used to calibrate the IMU.
<p align="center">
  <img  width="500" alt="image" src="https://github.com/user-attachments/assets/925bb670-5b05-4bd7-b940-b64c5fa9b121" />
  <img width="500"  alt="image" src="https://github.com/user-attachments/assets/8a2e3ac7-c857-44e2-b5f2-0fc4833805ae" />
</p>


5. First calibrate the gyro, place the tool flat on the table and press the **Calibrate Gyro**: button. Keep the tool still, do not touching it until a pop-up notifies that the calibration is complete.
6. <p> Next calibrate the accelerometer and the magnetometer this can be done at the same time since they both require the same movements. Hold the tool in the air, ensure it isn't touching anything. Press calibrate magnetometer then immediately press calibrate accelerometer. This calibration requires the IMU to be moved in full 360 degree rotation in three seperate planes. Start with the XY plane, ensure the tool is pointing into the monitor (perpendicular to the simulator and parallel to the ground) then very slowly rotate the knob of the tool until you see the shape of a full circle for the Acc XY and Mag XY graphs on the left of the visualization screen.
<p align="center">
  <img width="500"  alt="IMG_6091" src="https://github.com/user-attachments/assets/c296d4dc-f4ec-4b4b-bc0b-262e92be7345" />
</p>

Next rotate in the YZ orientation by starting directly in front of the simulator. Start pointing directly down then rotate parallel to the ground pointing left, then point directly up, then point right before finally pointing down

<p align="center">
  <img width="500" alt="IMG_6099" src="https://github.com/user-attachments/assets/7b39c446-3d35-4943-b90f-83a355eb0b5a" />
  <img width="500" alt="IMG_6100" src="https://github.com/user-attachments/assets/9784444e-2849-4a72-8be3-31a807a3dcd6" />
</p>

Next rotate in the XZ direction by starting directly in front of the simulator. Point the tool directly up then rotate in full 360 degrees going in the direction of the simulator, then point to the floor, then pointing away from the simulator parallel to the ground before finally pointing up again. A complete circle should be seen on the visualization tab.
<p align="center">
  <img width="500" alt="IMG_6092" src="https://github.com/user-attachments/assets/0e17cf97-791e-42da-af04-bf7b65ccd0d1" />
  <img width="500" alt="IMG_6093" src="https://github.com/user-attachments/assets/d7857f63-c7df-4ac9-9822-9f10f75af563" />
</p>

Pressed the finish calibration for both the accelerometer and the magnetometer, then press the save calibration button. The final calibration should appear as the image below: 
<p align="center">
  <img width="500" alt="image" src="https://github.com/user-attachments/assets/f3866d51-6c32-413e-bc41-9a861175784c" />
</p>

Note: If the calibration does not appear to have smooth lines, or does not resemble a circle or oval you can do the calibration again by pressing calibrate magnetometer then immediately press calibrate accelerometer.
</p >

7. Repeat steps 3 to 6 for the other IMU by unplugging the one that was just calibrated and plugging in the new one. Ensure to give the new IMU a different name, otherwise the older calibration values will be overwritten.
8. Plug in both IMUs. Run `sensor_data/stream_sensors/stream_sensors.ino` on the Arduino and ensure all sensors are streaming.
9. To compute the offset of the simulator run `.\venv\Scripts\python.exe calibration/get_quaternion_offset.py` and place the tools as in the image below. Once ready press enter, note that there is a few seconds of delay between when enter is pressed and calibration begins to ensure that a single user has enough time to press a button and get in the sensors in position.

<p align="center">
  <img width="500" alt="IMG_6098 2" src="https://github.com/user-attachments/assets/a1aa63ed-ce64-4464-b918-b41d4ed7b115" />
</p>

9. To verify calibration is correct run `.\venv\Scripts\python.exe diagnostics/visualization/dual_tool_orientation.py` 

### TOF Calibration 
- TODO

## Hardware
Contains code for IMU (MinIMU-9) and ToF (VL53L0X) sensors as well as Load Cell Amp (HX711) interfacing with Arduino Uno

<p align="center">
  <img width="476" height="541" alt="image" src="https://github.com/user-attachments/assets/9358fd2e-0a77-4c9c-b920-438085ae9098" />
</p>
