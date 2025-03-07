// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/DigitalInput.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>

#include <rev/SparkMax.h>

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  frc2::CommandPtr raiseElevatorCommand();
  frc2::CommandPtr lowerElevatorCommand();
  frc2::CommandPtr coralOutCommand();
  frc2::CommandPtr coralInCommand();
  frc2::CommandPtr shootCoralCommand();
  frc2::CommandPtr automaticCoralGrabCommand();

  frc2::CommandPtr firstPositionCommand();
  frc2::CommandPtr secondPositionCommand();
  frc2::CommandPtr thirdPositionCommand();
  frc2::CommandPtr fourthPositionCommand();

  units::meter_t elevatorPosition = 0.0_m;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  

  frc::ProfiledPIDController<units::meters> m_elevatorPID{0.093383, 0.0, 0.0, {70_mps, 70_mps_sq}};
  frc::ElevatorFeedforward m_elevatorFF{0.24579_V, 0.26331_V, 0.12691_V / 1_mps, 0.02155_V / 1_mps_sq};

  rev::spark::SparkMax m_elevatorMotor{OperatorConstants::k_elevator_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_coralMotor{OperatorConstants::k_coral_id, rev::spark::SparkMax::MotorType::kBrushless};
  frc::DigitalInput m_coralSensor{OperatorConstants::k_sensor_id};
  frc::DigitalInput m_coralBarrier{OperatorConstants::k_barrier_id};
};
