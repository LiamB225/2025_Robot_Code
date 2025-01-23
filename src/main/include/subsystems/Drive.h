// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define _USE_MATH_DEFINES

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <math.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include <rev/SparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <ctre/phoenix6/CANcoder.hpp>

class Drive : public frc2::SubsystemBase {
 public:
  Drive();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr driveCommand(
    std::function<double(void)> drive_power,
    std::function<double(void)> strafe_power,
    std::function<double(void)> rot_power
  );

 private:
  units::meters_per_second_t kMaxSpeed = 2_mps;
  units::radians_per_second_t kRotMaxSpeed = 4_rad_per_s;
  units::radians_per_second_squared_t kRotMaxAccel = 3_rad_per_s_sq;

  units::meter_t kWheelBase = 0.6985_m;
  units::meter_t kTrackWidth = 0.6477_m;

  frc::SwerveDriveKinematics<4> m_DriveKinematics{
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}
  };

  //FrontLeftModule
  rev::spark::SparkMax m_flDriveMotor {OperatorConstants::k_fl_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_flRotMotor {OperatorConstants::k_fl_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_flRotEncoder {OperatorConstants::k_fl_rot_encoder, "rio"};
  frc::PIDController m_flDrivePID {0.0020645, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_flRotPID {0.01, 0.0, 0.0, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_flDriveFF {0_V, 0_V * 1_s / 1_m, 0_V * 1_s * 1_s / 1_m};
  frc::SimpleMotorFeedforward<units::radians> m_flRotFF {0_V, 0_V * 1_s / 1_rad, 0_V * 1_s * 1_s / 1_rad};
  rev::spark::SparkAbsoluteEncoder m_flDriveEncoder = m_flDriveMotor.GetAbsoluteEncoder();

  //FrontRightModules
  rev::spark::SparkMax m_frDriveMotor {OperatorConstants::k_fr_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_frRotMotor {OperatorConstants::k_fr_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_frRotEncoder {OperatorConstants::k_fr_rot_encoder, "rio"};
  frc::PIDController m_frDrivePID {0.0020645, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_frRotPID {0.01, 0.0, 0.0, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_frDriveFF {0_V, 0_V * 1_s / 1_m, 0_V * 1_s * 1_s / 1_m};
  frc::SimpleMotorFeedforward<units::radians> m_frRotFF {0_V, 0_V * 1_s / 1_rad, 0_V * 1_s * 1_s / 1_rad};
  rev::spark::SparkAbsoluteEncoder m_frDriveEncoder = m_frDriveMotor.GetAbsoluteEncoder();

  //BackLeftModule
  rev::spark::SparkMax m_blDriveMotor {OperatorConstants::k_bl_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_blRotMotor {OperatorConstants::k_bl_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_blRotEncoder {OperatorConstants::k_bl_rot_encoder, "rio"};
  frc::PIDController m_blDrivePID {0.0020645, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_blRotPID {0.01, 0.0, 0.0, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_blDriveFF {0_V, 0_V * 1_s / 1_m, 0_V * 1_s * 1_s / 1_m};
  frc::SimpleMotorFeedforward<units::radians> m_blRotFF {0_V, 0_V * 1_s / 1_rad, 0_V * 1_s * 1_s / 1_rad};
  rev::spark::SparkAbsoluteEncoder m_blDriveEncoder = m_blDriveMotor.GetAbsoluteEncoder();

  //BackRightModule
  rev::spark::SparkMax m_brDriveMotor {OperatorConstants::k_br_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_brRotMotor {OperatorConstants::k_br_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_brRotEncoder {OperatorConstants::k_br_rot_encoder, "rio"};
  frc::PIDController m_brDrivePID {0.0020645, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_brRotPID {0.01, 0.0, 0.0, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_brDriveFF {0_V, 0_V * 1_s / 1_m, 0_V * 1_s * 1_s / 1_m};
  frc::SimpleMotorFeedforward<units::radians> m_brRotFF {0_V, 0_V * 1_s / 1_rad, 0_V * 1_s * 1_s / 1_rad};
  rev::spark::SparkAbsoluteEncoder m_brDriveEncoder = m_brDriveMotor.GetAbsoluteEncoder();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
