// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "Constants.h"
#include "subsystems/Drive.h"
#include "subsystems/Elevator.h"
#include "subsystems/Climber.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  frc::SendableChooser<frc2::Command*> autoChooser;
  
  

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{
    OperatorConstants::kDriverControllerPort
  };
  frc2::CommandXboxController m_secondaryController{
    OperatorConstants::kSecondaryControllerPort
  };

  // The robot's subsystems are defined here...
  Drive m_drive;
  Elevator m_elevator;
  Climber m_climber;

  void ConfigureBindings();
};
