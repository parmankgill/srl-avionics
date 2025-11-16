# srl-avionics
Avionics and flight software development for CU Boulder Sounding Rocket Lab (SRL) vehicles, including Mamba III and Spaceshot platform. Features autonomous event detection, attitude determination infrastructure, and finite state machine architecture for high-altitude vehicle operations.
## Project Overview
This repository documents flight-critical avionics systems developed for high-performance sounding rockets. The primary focus is on attitude determination, autonomous state management, and recovery deployment sequencing for vehicles operating at extreme altitudes and velocities.
**Mamba III** served as a validation platform, achieving 90,000 feet and Mach 3.4 on a P-class motor. Flight data from this mission directly informed system improvements and validated our simulation models.
**Spaceshot** is the next evolution, targeting the Kármán line with a significantly larger motor and enhanced avionics architecture.

## System Architecture
### Core Subsystems
**Attitude Determination System**
- Extended Kalman Filter (EKF) for 6-DOF attitude estimation
- Coordinate frame transformations (LLA/ECEF/ECI/ENU) with GAST-based Earth rotation modeling
- Inertial effect corrections for high-altitude operations

**Flight State Machine**
- Autonomous state detection and transitions
- Telemetry downlink management
- Recovery deployment sequencing with event-driven logic

**Event Detection System**
- Real-time apogee and deployment event identification
- Sensor fusion validation for reliable triggers
- Redundant detection mechanisms

## Mamba III Flight Mission
**Mission Profile**
- Launch Date: October 12, 2025
- Apogee: 90,000 feet
- Peak Velocity: Mach 3.4
- Flight Duration: ~10 minutes

[![Mamba III Launch](https://img.youtube.com/vi/oLwV8E7ryng/maxresdefault.jpg)](https://www.youtube.com/watch?v=oLwV8E7ryng)

**Contributions**
- Designed and implemented event detection system within flight software
- Developed coordinate frame transformation algorithms (LLA/ECEF/ECI/ENU) with GAST-based rotation modeling and inertial effect corrections

**Flight Data Validation**
Two of three flight systems successfully gathered usable data, allowing characterization of:
- **Sensor saturation behavior**: Characterized accelerometer (BMI270) and barometric pressure sensor (BMP390) saturation limits at extreme flight conditions, informing component selection for higher-altitude Spaceshot missions
- **GPS Lockout Performance**: Documented GPS signal loss during ascent and recovery at ~12 km MSL, providing empirical data for inertial propagation algorithms during GPS-denied flight phases
- **Barometric Altitude Determination**: Compared barometric measurements against Kate-3 transmitter ground truth, confirming altitude accuracy and validating cavity venting design for reliable pressure-based measurements
- **Kalman Filter Performance**: Validated 6-DOF attitude determination and sensor fusion algorithms; empirical natural frequency response matched simulations across the flight envelope, demonstrating robust estimation even with sensor anomalies

## Looking Forward: Spaceshot
**Enhanced Avionics Architecture**
Building on Mamba III validation, the Spaceshot avionics system will scale to support operation at the Kármán line with significantly improved sensor redundancy and fault tolerance.

**Contributions**
Currently developing enhanced flight software for Spaceshot:
- Leading design of next-generation finite state machine architecture with fault detection and recovery modes
- Developing refined coordinate frame transformation algorithms to handle edge cases identified during Mamba III ascent
- Implementing sensor redundancy and voting logic for GPS-denied sections of flight
- Designing telemetry uplink protocols optimized for high-altitude communication constraints
- Architecting recovery deployment sequencing with multiple event validation triggers

