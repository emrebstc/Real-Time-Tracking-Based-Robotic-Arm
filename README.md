#  STM32 Sensor Fusion Robotic Arm Control (Real-Time Object Tracking)

This project implements embedded software and AI detection for a **5 Degrees of Freedom (DOF) robotic arm** controlled by an STM32F407 microcontroller. The goal is to enable the arm to autonomously **track** a designated object and **position itself** at a precise target distance using a **sensor fusion** approach.

I shared two versions, one made with functions and the other made with freeRTOS.

##  Project Goal

* It is a project based on embedded systems and artificial intelligence, designed to understand freeRTOS and various algorithms. It is a project designed for learning purposes, and the main purpose is to create a project that uses all these systems together.

* The primary goal is to lock the arm's horizontal (pan) and vertical (tilt) axes onto the target using image processing (camera) data, while simultaneously maintaining a constant distance (e.g., 5 cm) from the object by controlling the arm's reach (proximity) using data from the ultrasonic distance sensor. In this way, the robotic arm attempts to reach the target and perform the gripping motion. After detecting the center point of the target, it begins the necessary movements to reach it.

##  Key Control Mechanisms

The control is executed in the main loop using **non-blocking, step-by-step** functions with interrupts that continuously adjust the servo angles based on error feedback.
* **Primary Goal**	To autonomously lock the arm onto a target using Vision (DX/DY) and maintain a set distance using Ultrasonic Sensor data.	STM32F4, Raspberry Pi 5
* **Control Type**	Incremental (Step-by-Step) control. Movement speed is proportional to the error magnitude. C Language (STM32)
* **Vision Role**	Runs Python/YOLOv8n to detect the target, calculate DX/DY coordinates, and transmit them via UART.	Raspberry Pi 5
* **Control Role**	Consumes DX/DY and raw distance data to generate precise PWM signals for the servos.	STM32F407VGT6

##  Implementation Details

* **Control Type:** Incremental movement control (smooth stepping). Movement speed scales with the size of the error.
* **Input:** Camera (Receiving DX, DY coordinates via UART) and Ultrasonic Sensor (TIM Input Capture).
* **Output:** PWM control for the Servo motors.
* **Software:** Developed using C and Python languages also with STM32 HAL Drivers.

##  Hardware and Tools Used

This project involves a sensor fusion architecture, utilizing the processing power of a single-board computer for vision and a microcontroller for precise motor control. 

* **Microcontroller:** **STM32F407VGT6** Development Board (Handles PWM generation and Ultrasonic timing).
* **Vision Processing Unit (VPU):** **Raspberry Pi 5** (Used for running the vision algorithm, detecting the object via yolov8n.pt, and sending the processed DX/DY coordinates to the STM32 via UART/Serial Communication).
* **Actuators (Servos):** **6x MG996R** Metal Gear Servos (Used for moving the joints of the robotic arm).
* **Camera Input:** **1x USB Camera** (Provides the visual feed to the Raspberry Pi for object detection).
* **Distance Sensor:** **1x HC-SR04** Ultrasonic Sensor (Provides real-time distance measurements for the proximity control of Servo 5).
* **Interface:** **1x Logic Level Converter** (Essential for safe communication between the 3.3V STM32/Raspberry Pi logic and other components, if needed, or specifically for connecting the Raspberry Pi (1.8V/3.3V) to 5V servos or sensors if not directly compatible).

## 🛠️ Resources used

* https://github.com/ultralytics/ultralytics  - Yolov8n or Yolov5nu from Ultralytics
* https://docs.ultralytics.com/tr/models/yolov5/#performance-metrics 
* https://downloads.raspberrypi.com/raspios_lite_arm64/images/  - Raspberry Pi OS Lite
* https://www.raspberrypi.com/software/ -  Raspberry Pi Imager
* https://www.st.com/en/development-tools/stm32cubeide.html - STM32CubeIDE 1.19.0
* https://onnxruntime.ai/docs/performance/model-optimizations/quantization.html - onnx file quantization quide
  
## Example

https://github.com/user-attachments/assets/c546d577-b156-47b1-aba9-77057f3d7076

![1](https://github.com/user-attachments/assets/07d55908-e288-49ff-83a2-9dd3886d27fd)

![5](https://github.com/user-attachments/assets/b35c4ec6-c2f5-440e-b696-ef715a30097d)

![8](https://github.com/user-attachments/assets/a496d430-7ca1-44a0-8f62-55f8a21e27a7)

![2](https://github.com/user-attachments/assets/c1678fa8-820e-47e3-980e-b0647dd59533)

![4](https://github.com/user-attachments/assets/eb2684df-f0fd-4bee-891d-0c92902e0025)

![3](https://github.com/user-attachments/assets/541096cc-e399-48e0-8b54-a19156c07dac)


################################################################################################
* The PID algorithm can also be integrated by making various servo modifications. To test this, I connected a cable to the internal potentiometer of the servos as seen in the picture, so it is possible to get analog feedback.

![Modify](https://github.com/user-attachments/assets/cf1bcfe5-4560-4166-9c67-ad90d7a73c89)

