// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  pathplanner::NamedCommands::registerCommand("FourthElevatorCommand", m_elevator.fourthPositionCommand());
  pathplanner::NamedCommands::registerCommand("FirstElevatorCommand", m_elevator.firstPositionCommand());
  pathplanner::NamedCommands::registerCommand("IntakeInCommand", m_elevator.coralInCommand().WithTimeout(0.5_s));

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

  m_driverController.RightTrigger().WhileTrue(m_elevator.coralOutCommand());
  m_driverController.LeftTrigger().WhileTrue(m_elevator.coralInCommand());
  
  // m_driverController.LeftBumper().WhileTrue(frc2::cmd::Parallel(m_drive.ScoreLeftCommand([this]() {
  //     if(m_elevator.elevatorPosition == 100.0_m) {
  //       return -0.70;
  //     }
  //     else if(m_elevator.elevatorPosition == 60.0_m) {
  //       return -0.52;
  //     }
  //     else {
  //       return -0.46;
  //     } 
  //   }), frc2::cmd::RunOnce([this]() {m_drive.RemoveDefaultCommand();})));
  // m_driverController.LeftBumper().OnFalse(frc2::cmd::RunOnce([this]() {m_drive.SetDefaultCommand(m_drive.driveCommand(
  //   [this]() { return -m_driverController.GetLeftY(); },
  //   [this]() { return -m_driverController.GetLeftX(); },
  //   [this]() { return -m_driverController.GetRightX(); }
  // ));}));

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
