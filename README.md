# RC-Fire-Fighter-Esp-8266-
This Repository contains code to Run RC Fire Fighter Robot on ESP(8266)
Project Overview

This project is an RC Fire-Fighting Robot designed to autonomously detect and extinguish fires while being controllable via a PS4 controller. The robot integrates real-time AI-based fire detection with remote control capabilities, providing a smart solution for emergency scenarios.

System Components

PS4 Controller

Used to manually control the robot via a laptop interface.

Connects to the laptop through Bluetooth or USB.

ESP8266 Wi-Fi Module

Acts as the communication bridge between the laptop and the robot.

Receives commands from the laptop to control the pump for fire suppression.

ESP32-CAM

Captures live video feed from the robot’s environment.

Streams video to the laptop for real-time processing.

Laptop with Python & YOLO AI

Runs Python scripts to manage communication between PS4, ESP modules, and robot.

Performs real-time fire detection using YOLO AI on the video feed.

Sends signals to ESP8266 to activate the pump when fire is detected.

Relay & Pump

Relay module receives control signals from ESP8266 to turn the pump on or off.

Pump sprays water to extinguish detected fires.

System Workflow

Robot Control:

The PS4 controller connects to the laptop.

Commands from the controller are processed via Python scripts.

Movement commands are sent from the laptop to the robot via ESP8266.

Fire Detection:

ESP32-CAM streams live video to the laptop.

YOLO AI analyzes the video feed for fire detection.

Fire Suppression:

Upon fire detection, the laptop sends a signal to ESP8266.

ESP8266 triggers the relay to turn on the pump.

The pump sprays water to extinguish the fire.

Requirements

Hardware:

PS4 Controller

ESP8266 Module

ESP32-CAM Module

Relay Module

Water Pump

RC Robot Chassis

Software:

Python 3.x

YOLO Fire Detection Model

PySerial (for communication with ESP8266)

OpenCV (for video streaming and processing)

Required Python libraries for PS4 controller integration

Installation

Clone this repository:

git clone <repository_url>


Install Python dependencies:

pip install -r requirements.txt


Connect PS4 controller to the laptop.

Connect ESP8266 and ESP32-CAM modules to the laptop.

Run the main Python script:

python main.py


Ensure YOLO AI is running for fire detection.

Usage

Use the PS4 controller to navigate the robot.

Monitor the video feed from ESP32-CAM on your laptop.

When fire is detected, the pump automatically activates via ESP8266.

Optionally, manually control the pump using the controller if needed.

Features

Real-time fire detection using AI (YOLO).

Remote control using PS4 controller.

Automatic water pump activation on fire detection.

Wireless communication between laptop and robot.

Future Enhancements

Autonomous navigation to approach fire sources without manual control.

Integration of multiple sensors for obstacle avoidance.

Port YOLO AI to run directly on embedded hardware for faster detection.

Author

Zahid Ashraf
