# AI-Based Waste Detection and Collection Robot

This repository contains the Arduino and Python source code developed for the final-year project titled **AI-Based Waste Detection and Collection Robot**.

## Project Overview

The robot uses a YOLO11n object detection model to detect plastic bottles and aluminium cans. A laptop performs the object detection process and sends movement commands to an ESP32 through serial communication.

## Repository Structure

- `arduino/robot_controller.ino` — Controls the motors, ultrasonic sensors and robotic arm.
- `python/waste_detection_controller.py` — Performs YOLO object detection, target alignment and serial communication.
- `requirements.txt` — Lists the required Python libraries.

## Hardware

- ESP32
- Four JGB37 geared motors
- Two BTS7960 motor drivers
- HC-SR04 ultrasonic sensors
- Five-degree-of-freedom robotic arm
- DJI Osmo Action 4
- Laptop for YOLO inference

## Software Requirements

- Python 3
- Arduino IDE
- Ultralytics YOLO
- OpenCV
- PySerial

## Installation

Install the required Python packages:

```bash
pip install -r requirements.txt
