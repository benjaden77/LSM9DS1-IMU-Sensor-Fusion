# LSM9DS1 IMU Sensor Fusion & BLE Streaming

An embedded C++ project focused on inertial sensor calibration, signal processing, orientation estimation, and wireless data transmission using the **LSM9DS1 9-axis IMU** and an **nRF52-based development environment**.

The project explores the complete processing pipeline from raw sensor measurements to orientation estimation and motion data streaming.

## Overview

Inertial Measurement Units (IMUs) are commonly used in wearable devices, motion tracking, robotics, and embedded sensing systems.

However, raw sensor measurements are affected by noise, sensor bias, coordinate-frame differences, and sampling irregularities.

This project implements an IMU data processing pipeline to address these engineering challenges.

The system collects accelerometer, gyroscope, and magnetometer measurements through I²C, applies calibration and filtering, estimates orientation using the Mahony AHRS algorithm, and transmits processed data over Bluetooth Low Energy (BLE).

## Hardware and Technologies

**Hardware**
- LSM9DS1 9-axis IMU
- nRF52-based development board
- I²C communication interface
- Bluetooth Low Energy

**Software**
- C++ / Arduino
- SparkFun LSM9DS1 Library
- Adafruit Bluefruit nRF52
- Mahony AHRS Algorithm
- Quaternion-based Orientation Estimation
- First-order Low-pass Filtering

## System Architecture

The sensor data processing pipeline consists of the following stages:

1. **Sensor Acquisition:** Read accelerometer, gyroscope, and magnetometer measurements through I²C.
2. **Sensor Calibration:** Apply bias compensation and calibration matrices to correct measurement errors.
3. **Signal Filtering:** Reduce high-frequency measurement noise using a first-order low-pass filter.
4. **Sensor Fusion:** Estimate device orientation using Mahony AHRS and quaternion integration.
5. **Motion Processing:** Calculate directional vectors and gravity-compensated linear acceleration.
6. **BLE Transmission:** Package processed data into CSV-formatted records for external applications.

## Key Implementations

### 1. I²C Communication and Sensor Debugging

Implemented communication with the LSM9DS1 through the Arduino Wire interface.

The hardware configuration uses:
- Accelerometer/Gyroscope address: `0x6A`
- Magnetometer address: `0x1C`
- I²C clock: 400 kHz

An I²C scanning routine is also included to support device discovery and hardware troubleshooting.

### 2. Sensor Calibration

Accelerometer and magnetometer measurements are corrected using bias vectors and 3×3 calibration matrices.

The calibration model is:

**Corrected Data = Calibration Matrix × (Raw Data − Bias)**

A stationary gyroscope bias estimation routine is also included in the revised firmware to compensate for zero-rate offsets.

Calibration parameters are hardware-dependent and should be validated for each device.

### 3. Low-pass Filtering

A first-order low-pass filter is applied to reduce high-frequency measurement noise.

The filter coefficient can be adjusted dynamically based on the measured sampling interval, improving consistency when sensor update timing varies.

### 4. Mahony AHRS Sensor Fusion

The Mahony Attitude and Heading Reference System (AHRS) algorithm combines accelerometer, gyroscope, and magnetometer measurements to estimate three-dimensional orientation.

The system uses quaternions rather than directly integrating Euler angles.

Quaternion-based estimation supports continuous three-dimensional rotation while avoiding the singularities associated with Euler-angle representations.

### 5. Gravity Compensation

The estimated orientation is used to calculate the direction of gravity in the sensor coordinate frame.

The gravity component is then subtracted from the calibrated acceleration measurement to estimate linear acceleration.

The resulting motion acceleration is expressed in meters per second squared (m/s²).

### 6. BLE Data Streaming

The system uses the Adafruit Bluefruit BLE UART service to transmit processed motion data.

Each transmitted record contains 13 comma-separated values:

| Data | Fields |
|---|---|
| Linear acceleration | ax, ay, az |
| Angular velocity | gx, gy, gz |
| Direction vector | dx, dy, dz |
| Quaternion | qw, qx, qy, qz |

The configured transmission interval is 20 ms, corresponding to a target rate of approximately 50 Hz.

The actual throughput depends on sensor availability, processing time, and BLE communication conditions.

## Engineering Challenges

Several engineering considerations are involved in this implementation:

**Sensor Noise and Bias**

Raw measurements require calibration and filtering before they can be reliably used for orientation estimation.

**Coordinate Frame Consistency**

Sensor coordinate axes must be aligned consistently to avoid incorrect orientation calculations.

**Sampling and Timing**

Quaternion integration depends on the elapsed time between sensor updates. The firmware uses microsecond-resolution timing to calculate the update interval.

**Sensor Update Synchronization**

The accelerometer, gyroscope, and magnetometer may operate at different update rates. The revised processing pipeline supports independent magnetometer updates and cached measurements.

**Wireless Transmission**

The transmission interval must be balanced against data processing requirements and BLE communication capacity.

## Development Notes

This repository represents an embedded IMU processing and debugging implementation.

The calibration parameters, sensor axis conventions, and filter settings are dependent on the target hardware.

The code should be compiled and validated on the actual nRF52/LSM9DS1 setup before use in a motion-tracking application.

No specific orientation accuracy, latency, or packet-delivery performance is claimed without experimental measurement.

## Applications

The project provides a technical foundation for further work in:

- Wearable motion sensing
- Real-time orientation tracking
- Embedded sensor data processing
- Wireless motion data acquisition
- Motion analysis and human-computer interaction

## Summary

This project demonstrates practical embedded software development involving hardware communication, sensor calibration, digital signal processing, orientation estimation, and BLE data transmission.

It highlights the importance of measurement quality, timing consistency, coordinate-system conventions, and communication reliability when developing real-time sensing systems.
