# EV3 Master Project

Autonomous maze navigation scripts for LEGO Mindstorms EV3 robot.

## Hardware Configuration

- **Gyro Sensor**: Port 2 (mounted backwards)
- **Touch Sensor**: Port C (Port 3)
- **Ultrasonic Distance Sensor**: Port 4 (right side)
- **Right Wheel Motor**: Port A
- **Left Wheel Motor**: Port C
- ***WILL ADD OTHER SENSORS IN THE FUTURE***

## Files

### Main Navigation Scripts

- **MazeMapper_Gyro.m** - *new* gyro-assisted right-wall following with precise 90° turns (primary working solution)
- **MazeMapper.m** - *old* zone-based right-wall following without gyro

### Testing & Utilities

- **SensorTest.m** - real-time sensor monitoring and debugging
- **Beeper.m** - simple touch sensor test script
- **FrontDiffDrive_Brick.m** - differential drive control script

## Setup

1. Connect to EV3 brick via Bluetooth by running `connectEV3.m` from parent directory
2. Calibrate gyro sensor by placing robot on flat surface
3. Run desired navigation script

## Usage

```matlab
% From parent MATLAB directory
connectEV3

% Navigate to RobotNavigation folder and run
cd RobotNavigation
MazeMapper_Gyro
```

Press 'q' to stop any running navigation script.

## Navigation Algorithm

The gyro-assisted maze mapper uses:
- **Right-hand rule**: Always follows the right wall
- **Gyro heading control**: Maintains straight lines and precise 90° turns
- **Opening detection**: Turns right into pathways when no wall detected
- **Collision handling**: Backs up and turns left 90° when hitting front wall
- **Red line detection** (planned): Will use color sensor to detect and stop at red boundary lines for enhanced arena safety

## Authors

Daniel Allen, Nathan Alarcon, Thomas Stott, Ian Gallegos
