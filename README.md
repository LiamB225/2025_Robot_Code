# 2025 Robot Code - FRC Team 2143 Team Tobor

This repository contains the C++ robot code for the 2025 FIRST Robotics Competition (FRC) season game, **Reefscape**.

## 🤖 Robot Overview

The robot is built using the Command-Based framework and includes the following subsystems:

### Subsystems

*   **Drive**: Swerve drive drivetrain using REV Spark Max motor controllers and CTRE CANcoders.
    *   Features field-relative driving, automated scoring alignment (`ScoreLeft`, `ScoreRight`), and PathPlanner integration for autonomous navigation.
    *   Includes Vision processing with Limelight for pose estimation.
*   **Elevator**: Manipulator for handling Coral game pieces.
    *   Supports multiple setpoint positions (First, Second, Third, Fourth).
    *   Automated commands for grabbing and shooting Coral.
*   **Climber**: Mechanism for endgame climbing.

### Hardware

*   **Controllers:** 2x Xbox Controllers (Driver & Secondary)
*   **Sensors:** 
    *   ADIS16470 IMU
    *   Limelight 4 Vision Camera
    *   CTRE CANcoders (Swerve Rotation)
    *   Laser Barrier & Light Sensor
*   **Motors:**
    *   REV Spark Max Motor Controllers
    *   REV NEO Brushless Motors
    *   REV NEO 550 Brushless Motors

## 🎮 Controller Mappings

### Driver Controller (Port 0)

| Input | Action |
| :--- | :--- |
| **Left Stick Y** | Translation Forward/Backward |
| **Left Stick X** | Translation Left/Right |
| **Right Stick X** | Rotation |
| **Right Trigger** | Coral Out (While Held) |
| **Left Trigger** | Coral In (While Held) |
| **A Button** | Shoot Coral |
| **Left Bumper** | Align Score Left + Shoot Coral |
| **Right Bumper** | Align Score Right + Shoot Coral |
| **X Button** | Climb In (Manually) |
| **Y Button** | Climb Out (Manually) |
| **Start Button** | Automatic Climb |
| **B Button** | Automatic Climb Out |

### Secondary Controller (Port 1)

| Input | Action |
| :--- | :--- |
| **Right Bumper** | Raise Elevator (Manual) |
| **Left Bumper** | Lower Elevator (Manual) |
| **A Button** | Move to First Position |
| **B Button** | Move to Second Position |
| **X Button** | Move to Third Position |
| **Y Button** | Move to Fourth Position |

## 🏹 Autonomous

This project uses **PathPlanner** for autonomous routines.
*   **Named Commands** registered:
    *   `FourthElevatorCommand`
    *   `FirstElevatorCommand`
    *   `IntakeInCommand`
    *   `GrabCoralCommand`
    *   `DoNothing`

Select the desired auto routine using the Shuffleboard/SmartDashboard chooser.

## 🛠️ Build and Deploy

### Prerequisites
*   WPILib 2025.3.2 or later
*   Visual Studio Code with WPILib extension

### Building the Code
Open the project in VS Code and run the **WPILib: Build Robot Code** command (Shift+F5).

### Deploying to Robot
1.  Connect to the robot via USB or Wi-Fi.
2.  Run the **WPILib: Deploy Robot Code** command (Shift+F5).

---
*Author: Liam Bauer*
