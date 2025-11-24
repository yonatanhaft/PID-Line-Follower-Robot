# 🏎️ Autonomous PID Line Follower Robot

An autonomous line-following robot developed by Electrical Engineering students for the Semester B robotics competition. The robot utilizes a PID control algorithm to navigate a track with high precision and speed.

## 📌 Project Overview
The goal of this project was to design and program a robot capable of following a black line on a white surface, completing a complex track in minimum time.
The system is based on an **Arduino UNO** and uses a closed-loop control system (PID) to correct errors in real-time.

## 🛠️ Hardware Components
Based on the standard academic kit with a custom sensor array upgrade:

* **Controller:** Arduino UNO
* **Motor Driver:** L298N Dual H-Bridge
* **Sensors:** 5-Channel IR Line Tracking Sensor Module (upgraded from the standard 2 sensors)
* **Motors:** 2x DC Gear Motors (Standard Kit)
* **Chassis:** Acrylic 2WD Chassis with a caster wheel
* **Power:** Li-Ion Batteries

## ⚙️ Software & Control Logic
The code is written in C++ using the Arduino IDE.

### Key Features:
* **PID Control:** Implements Proportional (P), Integral (I), and Derivative (D) control to smooth out the movement and handle sharp turns.
    * `Kp = 1.65` (Proportional constant)
    * `Ki = 0.0` (Integral constant)
    * `Kd = 3.70` (Derivative constant)
* **Auto-Calibration:** The robot performs a calibration routine during the `setup()` phase to adapt to different lighting conditions and track surfaces.
* **Library Used:** `QTRSensors` for efficient sensor reading.

### Pin Configuration (Wiring):
| Component | Arduino Pin |
|-----------|-------------|
| Left Motor PWM | 3 |
| Left Motor Dir | 5, 6 |
| Right Motor PWM | 9 |
| Right Motor Dir | 10, 11 |
| Sensors (1-5) | A1, A2, A3, A4, A5 |

## 🚀 How to Run
1.  **Assembly:** Connect the components according to the pin configuration table above.
2.  **Upload:** Open `NOSEATOV_TAHAROT.ino` in Arduino IDE and upload it to the board.
3.  **Calibration:** * Place the robot on the line.
    * Turn it on. The robot will rotate/move the sensors over the line for ~400 cycles to calibrate black/white values.
    * **Note:** The LED on pin 13 indicates calibration mode.
4.  **Run:** After calibration (and a short delay), the robot will start following the line.

---
**Created by:** Yonatan Haftman.
