
# Admittance Controller for Exosuit Assistance

This repository implements an **admittance-based controller** designed to assist human motion using an exosuit. The controller is structured as a **PID-like admittance** with the form:

$$
\frac{f}{\omega} = P + Ds + \frac{I}{s}
$$

where $$f$$ is the cable tension, $$\omega$$ is the angular velocity of the motor, and $$P$$, $$D$$, and $$I$$ are the proportional, derivative, and integral gains, respectively.

---

## Features

### **1. Elbow Flexion Assistance**
The `Admittance_elbow_EMG` folder contains the controller for assisting elbow flexion. The detection method relies on **gravity compensation**, and joint torques are estimated using one of the following approaches:
- **IMU-based estimation**: Combines inertial measurement units (IMUs) with a static model.
- **sEMG-based estimation**: Uses surface electromyography (sEMG) signals and EMG-to-torque conversion models.

### **2. Shoulder Flexion Assistance**
The `Admittance_shoulderflex_EMG` folder provides the controller for assisting shoulder flexion.

### **Real-Time Implementation**
The code is optimized for **real-time execution** on an **STM32 Nucleo-F722ZE** microcontroller.

---
## Requirements

To run this project, you will need the following libraries:

1. **Adafruit_BNO055** – For IMU sensor (BNO055) communication.
2. **Adafruit_Sensor** – Base library for Adafruit sensors.
3. **Wire** – I2C communication library (included with Arduino IDE).
4. **STM32TimerInterrupt** – For precise timer-based interrupts on STM32.
5. **EMG_Filtering_STM32** – For real-time EMG signal filtering.
   - Download from: [Pgrnlt/EMG_Real_Time_Digital_Filtering](https://github.com/Pgrnlt/EMG_Real_Time_Digital_Filtering)
6. **mcp_can** – For CAN bus communication.
7. **SPI** – Serial Peripheral Interface library (included with Arduino IDE).

---

## Installation

1. **Install Libraries**:
   - Use the Arduino Library Manager to install `Adafruit_BNO055`, `Adafruit_Sensor`, `mcp_can`, and `STM32TimerInterrupt`.
   - Manually download **EMG_Filtering_STM32** from the GitHub repository and add `file.zip` to your Arduino libraries folder.

2. **Configure STM32 Board**:
   - Ensure your STM32 board is properly configured in the Arduino IDE with the correct board support package.

3. **Hardware Setup**:
   - Connect the IMU sensors (BNO055) via I2C.
   - Connect the EMG sensors to the specified analog pins (`A2`, `A3`, `A4`).
   - Connect the CAN bus module to the STM32.

4. **Upload the Code**:
   - Open the project in the Arduino IDE.
   - Select the correct board and port.
   - Upload the code to the STM32.

---

## Demo Video
A demonstration of the exosuit assistance in action is available below:


https://github.com/user-attachments/assets/4cb50827-204a-4083-b282-c6bd32778ed3


