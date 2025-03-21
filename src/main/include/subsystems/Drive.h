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
#include <frc/ADIS16470_IMU.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/Timer.h>
#include <networktables/StructArrayTopic.h>
#include <cmath>
#include <math.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

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

  frc2::CommandPtr ScoreRightCommand();
  frc2::CommandPtr ScoreLeftCommand();

  void resetPosition(frc::Pose2d m_pose);

  frc::ChassisSpeeds getRobotRelativeChassisSpeeds();

  void autoDrive(
    frc::ChassisSpeeds autospeeds
  );

  void SwerveDrive(
    units::meters_per_second_t xspeed,
    units::meters_per_second_t yspeed,
    units::radians_per_second_t rotspeed,
    bool fieldRelative
  );

 private:
  units::meters_per_second_t kRobotMaxSpeed = 3_mps;
  units::radians_per_second_t kRobotRotMaxSpeed = 3_rad_per_s;
  frc::SlewRateLimiter<units::scalar> XRateLimiter{5 / 1_s};
  frc::SlewRateLimiter<units::scalar> YRateLimiter{5 / 1_s};
  frc::SlewRateLimiter<units::scalar> RotRateLimiter{5 / 1_s};

  units::meters_per_second_t kMaxSpeed = 3_mps;
  units::radians_per_second_t kRotMaxSpeed = 9.42_rad_per_s;
  units::radians_per_second_squared_t kRotMaxAccel = 100_rad_per_s_sq;

  units::meter_t kWheelBase = 0.59055_m;
  units::meter_t kTrackWidth = 0.56515_m;

  frc::SwerveDriveKinematics<4> m_DriveKinematics{
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}
  };

  frc::ADIS16470_IMU gyro;

  //FrontLeftModule
  rev::spark::SparkMax m_flDriveMotor {OperatorConstants::k_fl_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_flRotMotor {OperatorConstants::k_fl_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_flRotEncoder {OperatorConstants::k_fl_rot_encoder, "rio"};
  frc::PIDController m_flDrivePID {0.71452, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_flRotPID {3.0, 0.0, 0.05, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_flDriveFF {0.42325_V, 3.0308_V / 1_mps, 0.45131_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> m_flRotFF {0.45_V, 0.2_V / 1_rad_per_s};

  //FrontRightModules
  rev::spark::SparkMax m_frDriveMotor {OperatorConstants::k_fr_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_frRotMotor {OperatorConstants::k_fr_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_frRotEncoder {OperatorConstants::k_fr_rot_encoder, "rio"};
  frc::PIDController m_frDrivePID {0.38831, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_frRotPID {3.0, 0.0, 0.05, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_frDriveFF {0.40994_V, 3.061_V / 1_mps, 0.50163_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> m_frRotFF {0.45_V, 0.2_V / 1_rad_per_s};

  //BackLeftModule
  rev::spark::SparkMax m_blDriveMotor {OperatorConstants::k_bl_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_blRotMotor {OperatorConstants::k_bl_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_blRotEncoder {OperatorConstants::k_bl_rot_encoder, "rio"};
  frc::PIDController m_blDrivePID {1.1032, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_blRotPID {3.0, 0.0, 0.05, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_blDriveFF {0.33652_V, 3.024_V / 1_mps, 0.29702_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> m_blRotFF {0.45_V, 0.2_V / 1_rad_per_s};

  //BackRightModule
  rev::spark::SparkMax m_brDriveMotor {OperatorConstants::k_br_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_brRotMotor {OperatorConstants::k_br_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_brRotEncoder {OperatorConstants::k_br_rot_encoder, "rio"};
  frc::PIDController m_brDrivePID {0.74162, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_brRotPID {3.0, 0.0, 0.05, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_brDriveFF {0.32866_V, 3.1497_V / 1_mps, 0.32601_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> m_brRotFF {0.4_V, 0.2_V / 1_rad_per_s};

  //Position Estimating
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator{m_DriveKinematics, frc::Rotation2d{}, {
    frc::SwerveModulePosition{(units::meter_t)(m_flDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_frDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_blDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_brDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)}
  }, frc::Pose2d{}};

  frc::Field2d m_field;

  nt::NetworkTableInstance ntinst;

  frc::PIDController m_translationXPID{1.5, 0.0, 0.0};
  frc::PIDController m_translationYPID{1.5, 0.0, 0.0};
  frc::PIDController m_rotationPID{1.5, 0.0, 0.0};

  double XGoal = 3.80;
  double YGoal = 5.39;
  double RotGoal = -1.0471975512;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};