# F1TENTH Hardware System

Complete hardware interface and drivers for F1TENTH racing car. Provides low-level control and sensor integration.

Part of the F1TENTH autonomous racing system - interfaces with physical hardware for real-world deployment.

## Quick Start

```bash
# Build (from workspace root)
colcon build --packages-select f1tenth_stack --symlink-install
source install/setup.bash

# Launch complete F1TENTH hardware stack
ros2 launch f1tenth_stack bringup_launch.py
```

## Enhanced Odometry System

**Upgraded with IMU-Enhanced EKF Fusion:**
- Combines wheel odometry with IMU data using Extended Kalman Filter
- Provides more accurate pose estimation than wheel-only odometry
- Compensates for wheel slip and improves heading accuracy
- Real-time sensor fusion for better localization performance

## Components

### Hardware Drivers
- **VESC Motor Controller**: Speed and steering control
- **Hokuyo LiDAR**: Laser range finder (URG node)
- **IMU**: Inertial measurement for enhanced odometry (EKF fusion)
- **Wheel Encoders**: Base odometry from motor feedback
- **Joystick Support**: Manual control interface

### Enhanced Odometry Stack
- **EKF Localization**: Fuses wheel odometry + IMU data
- **Improved Accuracy**: Better pose estimation than wheel-only
- **Slip Compensation**: Handles wheel slip and skidding
- **High-Frequency Updates**: Smooth odometry at sensor rates

### Control Systems
- **Ackermann Multiplexer**: Switches between control sources
- **Throttle Interpolation**: Smooth acceleration curves
- **Safety Systems**: Emergency stops and limits

### Teleop Options
- **Joy Teleop**: Xbox/PS4 controller support
- **Key Teleop**: Keyboard control interface

## Manual Control

### Keyboard Teleop
```bash
ros2 run key_teleop key_teleop
```

**Controls:**
- `w` - Forward
- `s` - Backward  
- `a` - Turn left
- `d` - Turn right
- `x` - Stop

### Joystick Control
```bash
# Joystick should work automatically with bringup launch
# Configure mapping in config/joy_teleop.yaml
```

## Integration with Autonomous System

**Launch Order for Real Car:**
1. **Hardware**: `ros2 launch f1tenth_stack bringup_launch.py`
2. Localization: `ros2 launch particle_filter_cpp localize_slam_launch.py`
3. Set initial pose in RViz
4. Planning: `ros2 launch lattice_planner_pkg lattice_planner.launch.py`
5. Control: `ros2 launch path_follower_pkg path_follower.launch.py sim_mode:=false`

## Key Topics

**Publishes:**
- `/scan` - LiDAR laser scan data
- `/odom` - Enhanced odometry (EKF-fused wheel + IMU)
- `/imu/data` - IMU measurements
- `/wheel_odom` - Raw wheel encoder odometry
- `/joy` - Joystick inputs

**Subscribes:**
- `/ackermann_drive` - Drive commands from autonomous system
- `/ackermann_cmd_*` - Multiple command sources (muxed)

## Configuration Files

- `config/vesc.yaml` - Motor controller parameters
- `config/sensors.yaml` - LiDAR and IMU settings  
- `config/mux.yaml` - Control multiplexer priorities
- `config/joy_teleop.yaml` - Joystick button mapping
- `config/ekf.yaml` - EKF sensor fusion parameters

## EKF Odometry Benefits

**Improvements over wheel-only odometry:**
- **Heading Accuracy**: IMU gyroscope provides precise angular velocity
- **Slip Detection**: Detects and compensates for wheel slip
- **Smooth Estimates**: Continuous pose updates at high frequency
- **Racing Performance**: Better accuracy during aggressive maneuvers

## Safety Features

- **Ackermann Mux**: Prioritized control switching
- **Dead Man's Switch**: Requires active joystick input
- **Speed Limits**: Configurable maximum velocities
- **Emergency Stop**: Immediate brake capability

## Useful Aliases

```bash
# Add to ~/.bashrc or ~/.zshrc
alias f110="ros2 launch f1tenth_stack bringup_launch.py"  
alias key="ros2 run key_teleop key_teleop"
alias joy="ros2 topic echo /joy"  # Monitor joystick

# Source your shell config
source ~/.bashrc  # or ~/.zshrc
```
