# Force PID Controller for Universal Robots

This project implements a hybrid compliance control system for Universal Robots (UR) collaborative robots, specifically designed for deburring processes. The system features an indirect hybrid position/force controller with adjustable parameters.

## Overview

The project focuses on developing a force control system that allows:
- Precise force control during deburring operations
- Adjustable control parameters for different materials and surfaces
- Real-time force monitoring and control
- Hybrid position/force control implementation

## Technical Implementation

### Hardware Requirements
- Universal Robot (UR10/UR5)
- Network connection to the robot (default IP: 192.168.5.1)
- RTDE interface enabled on port 30004

### Software Components

#### Force Control System
- Implementation of a PID controller for force control
- Sampling rate: 20Hz (RTDE communication)
- Configurable control parameters:
  - Proportional gain (Kp)
  - Integral gain (Ki)
  - Derivative gain (Kd)

#### Signal Processing
- Bessel low-pass filter implementation
  - Filter order: 2
  - Cutoff frequency: 1.732 Hz
  - Sampling frequency: 50 Hz
- Real-time force data filtering for noise reduction

#### Data Collection
The system collects and logs various robot states including:
- TCP forces and torques
- Joint positions and velocities
- Actual TCP pose and speed
- Control currents
- Filtered force measurements

### Key Features

1. **Adaptive Force Control**
   - Dynamic force reference setting
   - Adjustable PID gains for different applications
   - Real-time force error compensation

2. **Data Logging**
   - Comprehensive data collection for analysis
   - CSV file export for post-processing
   - Multiple sensor data streams recorded

3. **Safety Features**
   - Watchdog implementation for system monitoring
   - Controlled force application
   - Emergency stop capability

## Technical Details

### Kinematics
- Full implementation of UR robot kinematics
- Denavit-Hartenberg parameters for precise positioning
- Jacobian-based force/torque estimation

### Control Architecture
- Hybrid position/force control scheme
- Indirect force control through joint torque estimation
- Real-time parameter adjustment capability

### Signal Processing
- Bessel filter implementation for optimal phase response
- Real-time data filtering
- Noise reduction while maintaining system responsiveness

## Results and Performance

The system has been tested on:
- Flat surfaces
- Curved pipes
- Various material types

Performance metrics include:
- Force tracking accuracy
- Position control precision
- System response time

## Future Improvements

Potential areas for enhancement include:
- Advanced filtering techniques
- Machine learning-based parameter optimization
- Enhanced surface detection
- Improved force control algorithms

## Dependencies

- Python 3.x
- NumPy
- SciPy
- Matplotlib
- RTDE Interface
- Universal Robots RTDE library

## Usage

1. Configure the robot IP and port in the configuration file
2. Set up the RTDE interface
3. Adjust control parameters as needed
4. Run the control script
5. Monitor and analyze the collected data

For detailed implementation and testing procedures, refer to the technical documentation.


