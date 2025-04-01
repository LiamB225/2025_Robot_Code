// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <rev/SparkMax.h>

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  frc2::CommandPtr ClimbCommand();
  frc2::CommandPtr ClimbOutCommand();
  frc2::CommandPtr ClimbIn();
  frc2::CommandPtr ClimbOut();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::spark::SparkMax m_climberMotor{OperatorConstants::k_climber_id, rev::spark::SparkMax::MotorType::kBrushless};
  frc::PIDController m_climberPID{0.5, 0.0, 0.0};
};
