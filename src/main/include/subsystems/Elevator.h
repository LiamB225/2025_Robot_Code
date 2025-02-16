// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/SparkMax.h>

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  frc2::CommandPtr raiseElevatorCommand();
  frc2::CommandPtr lowerElevatorCommand();
  frc2::CommandPtr coralOutCommand();
  frc2::CommandPtr coralInCommand();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkMax m_elevatorMotor {OperatorConstants::k_elevator_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_coralMotor {OperatorConstants::k_coral_id, rev::spark::SparkMax::MotorType::kBrushless};
};
