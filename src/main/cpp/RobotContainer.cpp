// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  //pathplanner::NamedCommands::registerCommand("TestCommand", m_elevator.coralInCommand());

  autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_drive.SetDefaultCommand(m_drive.driveCommand(
    [this]() { return -m_driverController.GetLeftY(); },
    [this]() { return -m_driverController.GetLeftX(); },
    [this]() { return -m_driverController.GetRightX(); }
  ));

  m_driverController.RightBumper().WhileTrue(m_elevator.raiseElevatorCommand());
  m_driverController.LeftBumper().WhileTrue(m_elevator.lowerElevatorCommand());
  
  m_driverController.RightTrigger().WhileTrue(m_elevator.coralOutCommand());
  m_driverController.LeftTrigger().WhileTrue(m_elevator.coralInCommand());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}
