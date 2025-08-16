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
- **Ackermann Multiplexer**: Priority-based control switching between sources
  - **Keyboard Control** (`/keyboard_teleop`): Priority 100 (highest)
  - **Navigation** (`/drive`): Priority 10 (lower)
  - **Timeout Protection**: 0.2s timeout prevents stale commands
- **Throttle Interpolation**: Smooth acceleration curves
- **Safety Systems**: Emergency stops and priority-based control switching

### Teleop Options
- **Enhanced Key Teleop**: Multi-mode keyboard control with emergency stop
- **Joy Teleop**: Xbox/PS4 controller support (legacy)

## Manual Control

### Enhanced Keyboard Teleop
```bash
// It is available on devel branch. Not master branch
ros2 run key_teleop key_teleop
```

**Three Control Modes:**
- **🔵 'D' - Driving Mode**: Disables manual control, allows autonomous navigation
- **🟢 'C' - Control Mode**: Full manual control with speed profiles and progressive acceleration
- **🟡 'S' - Simulation Mode**: Publishes Twist messages to `/cmd_vel` for simulation testing

**Movement Controls:**
- **Arrow Keys**: `↑` Forward, `↓` Backward, `←` Left, `→` Right
- **Speed Profiles**: `1`-`0` keys (0.6 m/s to 4.0 m/s max speed)
- **🚨 Emergency Stop**: `SPACEBAR` - Immediate stop with zero commands

**Safety Features:**
- **Emergency Stop**: Instant zero velocity, maintains high-priority control
- **Progressive Acceleration**: Smooth speed ramping for better control
- **Mode-based Control**: Clear separation between manual/auto/simulation modes
- **Visual Feedback**: Color-coded interface with real-time status

**Key Features:**
- **Continuous Publishing**: In control mode, always publishes commands to maintain mux priority
- **Background Operation**: Can run while using other applications
- **Resume from Emergency**: Press any arrow key to resume from emergency stop

### Joystick Control
```bash
# Joystick should work automatically with bringup launch
# Configure mapping in config/joy_teleop.yaml
```

## Integration with Autonomous System

**Launch Order for Real Car:**
1. **Hardware**: `ros2 launch f1tenth_stack bringup_launch.py`
2. **Manual Control**: `ros2 run key_teleop key_teleop` (in separate terminal)
3. Localization: `ros2 launch particle_filter_cpp localize_slam_launch.py`
4. Set initial pose in RViz
5. Planning: `ros2 launch lattice_planner_pkg lattice_planner.launch.py`
6. Control: `ros2 launch path_follower_pkg path_follower.launch.py sim_mode:=false`

**Control Workflow:**
- **Start in 'C' (Control) mode**: Manual keyboard control active
- **Switch to 'D' (Driving) mode**: Enable autonomous navigation  
- **Emergency Stop**: Hit `SPACEBAR` anytime for immediate stop
- **Resume**: Press arrow keys to return to manual control

## Key Topics

**Publishes:**
- `/scan` - LiDAR laser scan data
- `/odom` - Enhanced odometry (EKF-fused wheel + IMU)
- `/imu/data` - IMU measurements
- `/wheel_odom` - Raw wheel encoder odometry
- `/joy` - Joystick inputs

**Subscribes:**
- `/ackermann_drive` - Drive commands from autonomous system
- `/keyboard_teleop` - Manual keyboard control commands (priority 100)
- `/drive` - Navigation/autonomous commands (priority 10)

## Configuration Files

- `config/vesc.yaml` - Motor controller parameters
- `config/sensors.yaml` - LiDAR and IMU settings  
- `config/mux.yaml` - Control multiplexer priorities (keyboard vs navigation)
- `config/key_teleop.yaml` - Keyboard teleop configuration
- `config/ekf.yaml` - EKF sensor fusion parameters

## EKF Odometry Benefits

**Improvements over wheel-only odometry:**
- **Heading Accuracy**: IMU gyroscope provides precise angular velocity
- **Slip Detection**: Detects and compensates for wheel slip
- **Smooth Estimates**: Continuous pose updates at high frequency
- **Racing Performance**: Better accuracy during aggressive maneuvers

## Safety Features

- **Enhanced Ackermann Mux**: Priority-based control switching
  - **Manual Override**: Keyboard control (priority 100) always overrides navigation
  - **Timeout Safety**: 0.2s timeout prevents stale command execution
  - **Seamless Switching**: Switch between manual and autonomous without restart
- **Emergency Stop System**: 
  - **Instant Response**: Spacebar for immediate zero velocity
  - **High Priority**: Emergency commands override all other inputs
  - **Continuous Safety**: Keeps publishing zero commands to maintain control
- **Progressive Control**: Smooth acceleration prevents sudden movements
- **Mode Separation**: Clear distinction between driving/control/simulation modes

## Useful Aliases

```bash
# Add to ~/.bashrc or ~/.zshrc
alias f110="ros2 launch f1tenth_stack bringup_launch.py"  
alias key="ros2 run key_teleop key_teleop"
alias joy="ros2 topic echo /joy"  # Monitor joystick

# Source your shell config
source ~/.bashrc  # or ~/.zshrc
```
