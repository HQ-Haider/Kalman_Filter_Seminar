# Kalman Filtering for State Estimation in Intelligent Systems

## Project Overview
This project demonstrates the implementation of a Kalman Filter for estimating the orientation of an intelligent system using an ESP32-C3 microcontroller and an MPU6050 sensor.

The filter combines accelerometer and gyroscope measurements to produce smoother and more reliable Roll and Pitch angle estimates.

## Project Objectives
- Understand the fundamentals of Kalman Filtering.
- Reduce measurement noise from sensor readings.
- Combine accelerometer and gyroscope data using sensor fusion.
- Implement the algorithm on an ESP32-C3.
- Display filtered orientation angles on an OLED screen.

## Hardware Components
| Component | Description |
|---|---|
| ESP32-C3 | Main microcontroller |
| MPU6050 | Accelerometer and gyroscope sensor |
| OLED Display | Displays estimated Roll and Pitch |
| Push Button | User interaction |
| TTP223 Touch Sensor | Touch input |

## System Architecture

**MPU6050 → ESP32-C3 (Kalman Filter) → OLED Display**

The MPU6050 collects motion data. The ESP32-C3 processes the measurements using the Kalman Filter, and the estimated angles are displayed on the OLED screen.

## Kalman Filter Methodology

The algorithm operates through two main steps:

1. **Prediction:** Uses gyroscope measurements to predict the next angle.
2. **Update:** Uses accelerometer measurements to correct the predicted estimate.

This process is repeated continuously to improve the estimation.

## Implementation
The practical implementation includes:

- Gyroscope calibration to estimate sensor bias.
- Raw Roll and Pitch angle calculation.
- Kalman Filter prediction and update.
- Real-time display of filtered angles.

## Results
The experimental results demonstrate that the filtered Roll and Pitch measurements are smoother and more stable than the raw sensor readings, particularly when the system is stationary.

## Technologies Used
- ESP32-C3
- MPU6050
- Arduino IDE
- C/C++
- Kalman Filtering
- I2C Communication
- OLED Display

## Project Team
- **Zahraa Basim Hassn** – Introduction and Mathematical Background
- **Haider Hassan Mohammed** – Practical Implementation
- **Hawraa Haider Ali** – Seminar Design and Presentation

## References
1. R. E. Kálmán, 1960 – A New Approach to Linear Filtering and Prediction Problems.
2. Welch & Bishop, 2006 – An Introduction to the Kalman Filter.
3. Grewal & Andrews – Kalman Filtering: Theory and Practice with MATLAB.

---

**Academic Seminar Project – 2026**
