// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>


RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  pathplanner::NamedCommands::registerCommand("FourthElevatorCommand", m_elevator.fourthPositionCommand());
  pathplanner::NamedCommands::registerCommand("FirstElevatorCommand", m_elevator.firstPositionCommand());
  pathplanner::NamedCommands::registerCommand("IntakeInCommand", m_elevator.shootCoralCommand().WithTimeout(0.5_s));
  pathplanner::NamedCommands::registerCommand("DoNothing", m_drive.DoNothingCommand());
  pathplanner::NamedCommands::registerCommand("FollowAgain", m_drive.ClearOverridesCommand());

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

  m_elevator.SetDefaultCommand(m_elevator.automaticCoralGrabCommand());
  m_driverController.RightTrigger().WhileTrue(m_elevator.coralOutCommand());
  m_driverController.LeftTrigger().WhileTrue(m_elevator.coralInCommand());
  m_driverController.A().WhileTrue(m_elevator.shootCoralCommand());
  
  m_driverController.LeftBumper().WhileTrue(m_drive.ScoreLeftCommand().AndThen(m_elevator.shootCoralCommand()));
  m_driverController.RightBumper().WhileTrue(m_drive.ScoreRightCommand().AndThen(m_elevator.shootCoralCommand()));

  m_secondaryController.RightBumper().WhileTrue(m_elevator.raiseElevatorCommand());
  m_secondaryController.LeftBumper().WhileTrue(m_elevator.lowerElevatorCommand());

  m_secondaryController.A().OnTrue(m_elevator.firstPositionCommand());
  m_secondaryController.B().OnTrue(m_elevator.secondPositionCommand());
  m_secondaryController.X().OnTrue(m_elevator.thirdPositionCommand());
  m_secondaryController.Y().OnTrue(m_elevator.fourthPositionCommand());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}
