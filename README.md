
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

## Demo Video
A demonstration of the exosuit assistance in action is available below:


https://github.com/user-attachments/assets/4cb50827-204a-4083-b282-c6bd32778ed3


