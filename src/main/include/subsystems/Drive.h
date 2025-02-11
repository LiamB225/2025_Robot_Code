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
#include <networktables/StructArrayTopic.h>
#include <cmath>
#include <math.h>
#include <frc/DriverStation.h>

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
  units::meters_per_second_t kRobotMaxSpeed = 1_mps;
  units::radians_per_second_t kRobotRotMaxSpeed = 2_rad_per_s;
  frc::SlewRateLimiter<units::scalar> XRateLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> YRateLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> RotRateLimiter{3 / 1_s};

  units::meters_per_second_t kMaxSpeed = 3_mps;
  units::radians_per_second_t kRotMaxSpeed = 6.28_rad_per_s;
  units::radians_per_second_squared_t kRotMaxAccel = 10_rad_per_s_sq;

  units::meter_t kWheelBase = 0.6985_m;
  units::meter_t kTrackWidth = 0.6477_m;

  frc::SwerveDriveKinematics<4> m_DriveKinematics{
    frc::Translation2d{kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{kWheelBase / 2, -kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, kTrackWidth / 2},
    frc::Translation2d{-kWheelBase / 2, -kTrackWidth / 2}
  };

  nt::StructArrayPublisher<frc::SwerveModuleState> publisher = nt::NetworkTableInstance::GetDefault()
      .GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates").Publish();

  frc::ADIS16470_IMU gyro;

  //FrontLeftModule
  rev::spark::SparkMax m_flDriveMotor {OperatorConstants::k_fl_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_flRotMotor {OperatorConstants::k_fl_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_flRotEncoder {OperatorConstants::k_fl_rot_encoder, "rio"};
  frc::PIDController m_flDrivePID {0.93435, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_flRotPID {2.0, 0.0, 0.05, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_flDriveFF {0.34215_V, 2.8653_V / 1_mps, 0.46871_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> m_flRotFF {0.32_V, 0.2_V / 1_rad_per_s};

  //FrontRightModules
  rev::spark::SparkMax m_frDriveMotor {OperatorConstants::k_fr_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_frRotMotor {OperatorConstants::k_fr_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_frRotEncoder {OperatorConstants::k_fr_rot_encoder, "rio"};
  frc::PIDController m_frDrivePID {0.22855, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_frRotPID {2.0, 0.0, 0.05, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_frDriveFF {0.26506_V, 3.1448_V / 1_mps, 0.26282_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> m_frRotFF {0.3_V, 0.2_V / 1_rad_per_s};

  //BackLeftModule
  rev::spark::SparkMax m_blDriveMotor {OperatorConstants::k_bl_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_blRotMotor {OperatorConstants::k_bl_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_blRotEncoder {OperatorConstants::k_bl_rot_encoder, "rio"};
  frc::PIDController m_blDrivePID {0.098998, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_blRotPID {2.0, 0.0, 0.05, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_blDriveFF {0.26098_V, 3.0776_V / 1_mps, 0.22215_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> m_blRotFF {0.3_V, 0.2_V / 1_rad_per_s};

  //BackRightModule
  rev::spark::SparkMax m_brDriveMotor {OperatorConstants::k_br_drive_id, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_brRotMotor {OperatorConstants::k_br_rot_id, rev::spark::SparkMax::MotorType::kBrushless};
  ctre::phoenix6::hardware::CANcoder m_brRotEncoder {OperatorConstants::k_br_rot_encoder, "rio"};
  frc::PIDController m_brDrivePID {0.67879, 0.0, 0.0};
  frc::ProfiledPIDController<units::radians> m_brRotPID {2.0, 0.0, 0.05, {kRotMaxSpeed, kRotMaxAccel}};
  frc::SimpleMotorFeedforward<units::meters> m_brDriveFF {0.25784_V, 3.0909_V / 1_mps, 0.32694_V / 1_mps_sq};
  frc::SimpleMotorFeedforward<units::radians> m_brRotFF {0.25_V, 0.2_V / 1_rad_per_s};

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

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
