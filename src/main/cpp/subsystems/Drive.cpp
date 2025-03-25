// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

Drive::Drive() {
    m_flRotPID.EnableContinuousInput(-(units::radian_t)(M_PI), (units::radian_t)(M_PI));
    m_frRotPID.EnableContinuousInput(-(units::radian_t)(M_PI), (units::radian_t)(M_PI));
    m_blRotPID.EnableContinuousInput(-(units::radian_t)(M_PI), (units::radian_t)(M_PI));
    m_brRotPID.EnableContinuousInput(-(units::radian_t)(M_PI), (units::radian_t)(M_PI));
    m_rotationPID.EnableContinuousInput(-M_PI, M_PI);
    frc::SmartDashboard::PutData("FLROT PID Controller", &m_flRotPID);
    frc::SmartDashboard::PutData("FRROT PID Controller", &m_frRotPID);
    frc::SmartDashboard::PutData("BLROT PID Controller", &m_blRotPID);
    frc::SmartDashboard::PutData("BRROT PID Controller", &m_brRotPID);
    frc::SmartDashboard::PutData("FLDRIVE PID Controller", &m_flDrivePID);
    frc::SmartDashboard::PutData("FRDRIVE PID Controller", &m_frDrivePID);
    frc::SmartDashboard::PutData("BLDRIVE PID Controller", &m_blDrivePID);
    frc::SmartDashboard::PutData("BRDRIVE PID Controller", &m_brDrivePID);
    frc::SmartDashboard::PutData("Translation X PID", &m_translationXPID);
    frc::SmartDashboard::PutData("Translation Y PID", &m_translationYPID);
    frc::SmartDashboard::PutData("Rotation PID", &m_rotationPID);

    //PathPlanner AutoBuilder
    pathplanner::AutoBuilder::configure(
        [this]() {return m_poseEstimator.GetEstimatedPosition();},
        [this](frc::Pose2d autoPosition) {resetPosition(autoPosition);},
        [this]() {return getRobotRelativeChassisSpeeds();},
        [this](frc::ChassisSpeeds autoChassisSpeeds) {autoDrive(autoChassisSpeeds);},
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            pathplanner::PIDConstants(1.0, 0.0, 0.0),
            pathplanner::PIDConstants(1.0, 0.0, 0.0)
        ),
        pathplanner::RobotConfig::fromGUISettings(),
        []() {
            return false;
        },
        this
    );

    ntinst = nt::NetworkTableInstance::GetDefault();
    ntinst.GetTable("limelight-primary")->PutNumber("imumode_set", 2);

    m_translationXPID.SetTolerance(0.01);
    m_translationYPID.SetTolerance(0.01);
    m_rotationPID.SetTolerance(0.1);

    frc::SmartDashboard::PutData("field", &m_field);
}


// This method will be called once per scheduler run
void Drive::Periodic() {
    frc::Rotation2d angle{gyro.GetAngle()};
    m_poseEstimator.Update(angle, {
    frc::SwerveModulePosition{(units::meter_t)(m_flDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_frDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_blDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_brDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)}
    });

    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue && seedLimelight) {
        ntinst.GetTable("limelight-primary")->PutNumber("imumode_set", 1);
        m_poseEstimator.ResetRotation(frc::Rotation2d{180_deg});
        LimelightHelpers::SetRobotOrientation("limelight-primary", m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0);
        units::degree_t seedvalue = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-primary").pose.Rotation().Degrees();
        if(seedvalue > 170_deg || seedvalue < -170_deg) {
            ntinst.GetTable("limelight-primary")->PutNumber("imumode_set", 2);
            seedLimelight = false;
        }
    }

    if(ntinst.GetTable("limelight-primary")->GetNumber("tv", 0.0) == 1) {
        std::vector<double> pos1(6);
        pos1 = ntinst.GetTable("limelight-primary")->GetNumberArray("botpose_orb_wpiblue", std::vector<double>(6));
        frc::Pose2d limelightPosition1{(units::meter_t)(pos1[0]), (units::meter_t)(pos1[1]), frc::Rotation2d{(units::degree_t)(pos1[5])}};
        m_poseEstimator.AddVisionMeasurement(limelightPosition1, frc::Timer::GetTimestamp());
    }
    
    //std::vector<double> pos2(6);
    //pos2 = ntinst.GetTable("limelight-secondary")->GetNumberArray("botpose_wpiblue", std::vector<double>(6));
    //frc::Pose2d limelightPosition2{(units::meter_t)(pos2[0]), (units::meter_t)(pos2[1]), frc::Rotation2d{(units::degree_t)(pos2[5])}};
    //m_poseEstimator.AddVisionMeasurement(limelightPosition2, frc::Timer::GetTimestamp());

    m_field.SetRobotPose(m_poseEstimator.GetEstimatedPosition());
    frc::SmartDashboard::PutNumber("X", m_poseEstimator.GetEstimatedPosition().X().value());
    frc::SmartDashboard::PutNumber("Y", m_poseEstimator.GetEstimatedPosition().Y().value());
    frc::SmartDashboard::PutNumber("ROT", m_poseEstimator.GetEstimatedPosition().Rotation().Radians().value());
}


//Commands
frc2::CommandPtr Drive::driveCommand(
    std::function<double(void)> drive_power,
    std::function<double(void)> strafe_power,
    std::function<double(void)> rot_power
) {
    return this->Run(
        [this, drive_power, strafe_power, rot_power]() {
            units::meters_per_second_t drive_vel = XRateLimiter.Calculate(frc::ApplyDeadband(drive_power(), 0.07)) * kRobotMaxSpeed;
            units::meters_per_second_t strafe_vel = YRateLimiter.Calculate(frc::ApplyDeadband(strafe_power(), 0.07)) * kRobotMaxSpeed;
            units::radians_per_second_t rot_vel = RotRateLimiter.Calculate(frc::ApplyDeadband(rot_power(), 0.07)) * kRobotRotMaxSpeed;

            SwerveDrive(drive_vel, strafe_vel, rot_vel, true);
        }
    );
}


//Auto Scoring Commands
frc2::CommandPtr Drive::ScoreLeftCommand() {
    return this->StartRun(
        [this]() {
            double X = m_poseEstimator.GetEstimatedPosition().X().value();
            double Y = m_poseEstimator.GetEstimatedPosition().Y().value();

            if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
                if(X < 11.893677 && Y > 3.555492 && Y < 4.496308) {
                    //Red Middle Left
                    XGoal = 11.519706;
                    YGoal = 4.07;
                    RotGoal = 0.0;
                } else if(X < 11.893677 && Y > 4.496308) {
                    //Red Top Left
                    XGoal = 12.369452;
                    YGoal = 5.39;
                    RotGoal = -1.0471975512;
                } else if(X < 11.893677 && Y < 3.555492) {
                    //Red Bottom Left
                    XGoal = 12.278098;
                    YGoal = 2.7118;
                    RotGoal = 1.0471975512;
                } else if(X > 11.893677 && Y > 4.496308) {
                    //Red Top Right
                    XGoal = 13.839706;
                    YGoal = 5.34;
                    RotGoal = -2.09439510239;
                } else if(X > 11.893677 && Y < 3.555492) {
                    //Red Bottom Right
                    XGoal = 13.748352;
                    YGoal = 2.6618;
                    RotGoal = 2.09439510239;
                } else if(X > 11.893677 && Y > 3.555492 && Y < 4.496308) {
                    //Red Middle Right
                    XGoal = 14.598098;
                    YGoal = 3.9818;
                    RotGoal = 3.14159265359;
                }
            }
            else {
                if(X < 4.489323 && Y > 3.555492 && Y < 4.496308) {
                    //Blue Middle Left
                    XGoal = 2.95;
                    YGoal = 4.07;
                    RotGoal = 0.0;
                } else if(X < 4.489323 && Y > 4.496308) {
                    //Blue Top Left
                    XGoal = 3.80;
                    YGoal = 5.39;
                    RotGoal = -1.0471975512;
                } else if(X < 4.489323 && Y < 3.555492) {
                    //Blue Bottom Left
                    XGoal = 3.708646;
                    YGoal = 2.7118;
                    RotGoal = 1.0471975512;
                } else if(X > 4.489323 && Y > 4.496308) {
                    //Blue Top Right
                    XGoal = 5.27;
                    YGoal = 5.34;
                    RotGoal = -2.09439510239;
                } else if(X > 4.489323 && Y < 3.555492) {
                    //Blue Bottom Right
                    XGoal = 5.178646;
                    YGoal = 2.6618;
                    RotGoal = 2.09439510239;
                } else if(X > 4.489323 && Y > 3.555492 && Y < 4.496308) {
                    //Blue Middle Right
                    XGoal = 6.028646;
                    YGoal = 3.9818;
                    RotGoal = 3.14159265359;
                }
            }
            
        },
        [this]() {
            units::meters_per_second_t drive_vel = (units::meters_per_second_t)(m_translationXPID.Calculate(m_poseEstimator.GetEstimatedPosition().X().value(), XGoal));
            units::meters_per_second_t strafe_vel = (units::meters_per_second_t)(m_translationYPID.Calculate(m_poseEstimator.GetEstimatedPosition().Y().value(), YGoal));
            units::radians_per_second_t rot_vel = (units::radians_per_second_t)(m_rotationPID.Calculate(m_poseEstimator.GetEstimatedPosition().Rotation().Radians().value(), RotGoal));

            SwerveDrive(drive_vel, strafe_vel, rot_vel, true);
        }
    ).Until(
        [this]() {
            if(m_translationXPID.AtSetpoint() && m_translationYPID.AtSetpoint() && m_rotationPID.AtSetpoint()) {
                return true;
            }
            else {
                return false;
            }
        }
    );
}

frc2::CommandPtr Drive::ScoreRightCommand() {
    return this->StartRun(
        [this]() {
            double X = m_poseEstimator.GetEstimatedPosition().X().value();
            double Y = m_poseEstimator.GetEstimatedPosition().Y().value();

            if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
                if(X < 11.893677 && Y > 3.555492 && Y < 4.496308) {
                    //Red Middle Left
                    XGoal = 11.519706;
                    YGoal = 3.77;
                    RotGoal = 0.0;
                } else if(X < 11.893677 && Y > 4.496308) {
                    //Red Top Left
                    XGoal = 12.049452;
                    YGoal = 5.23;
                    RotGoal = -1.0471975512;
                } else if(X < 11.893677 && Y < 3.555492) {
                    //Red Bottom Left
                    XGoal = 12.508098;
                    YGoal = 2.5718;
                    RotGoal = 1.0471975512;
                } else if(X > 11.893677 && Y > 4.496308) {
                    //Red Top Right
                    XGoal = 13.609706;
                    YGoal = 5.48;
                    RotGoal = -2.09439510239;
                } else if(X > 11.893677 && Y < 3.555492) {
                    //Red Bottom Right
                    XGoal = 14.068352;
                    YGoal = 2.8218;
                    RotGoal = 2.09439510239;
                } else if(X > 11.893677 && Y > 3.555492 && Y < 4.496308) {
                    //Red Middle Right
                    XGoal = 14.598098;
                    YGoal = 4.2818;
                    RotGoal = 3.14159265359;
                }
            }
            else {
                if(X < 4.489323 && Y > 3.555492 && Y < 4.496308) {
                    //Blue Middle Left
                    XGoal = 2.95;
                    YGoal = 3.77;
                    RotGoal = 0.0;
                } else if(X < 4.489323 && Y > 4.496308) {
                    //Blue Top Left
                    XGoal = 3.48;
                    YGoal = 5.23;
                    RotGoal = -1.0471975512;
                } else if(X < 4.489323 && Y < 3.555492) {
                    //Blue Bottom Left
                    XGoal = 3.938646;
                    YGoal = 2.5718;
                    RotGoal = 1.0471975512;
                } else if(X > 4.489323 && Y > 4.496308) {
                    //Blue Top Right
                    XGoal = 5.04;
                    YGoal = 5.48;
                    RotGoal = -2.09439510239;
                } else if(X > 4.489323 && Y < 3.555492) {
                    //Blue Bottom Right
                    XGoal = 5.498646;
                    YGoal = 2.8218;
                    RotGoal = 2.09439510239;
                } else if(X > 4.489323 && Y > 3.555492 && Y < 4.496308) {
                    //Blue Middle Right
                    XGoal = 6.028646;
                    YGoal = 4.2818;
                    RotGoal = 3.14159265359;
                }
            }
            
        },
        [this]() {
            units::meters_per_second_t drive_vel = (units::meters_per_second_t)(m_translationXPID.Calculate(m_poseEstimator.GetEstimatedPosition().X().value(), XGoal));
            units::meters_per_second_t strafe_vel = (units::meters_per_second_t)(m_translationYPID.Calculate(m_poseEstimator.GetEstimatedPosition().Y().value(), YGoal));
            units::radians_per_second_t rot_vel = (units::radians_per_second_t)(m_rotationPID.Calculate(m_poseEstimator.GetEstimatedPosition().Rotation().Radians().value(), RotGoal));

            SwerveDrive(drive_vel, strafe_vel, rot_vel, true);
        }
    ).Until(
        [this]() {
            if(m_translationXPID.AtSetpoint() && m_translationYPID.AtSetpoint() && m_rotationPID.AtSetpoint()) {
                return true;
            }
            else {
                return false;
            }
        }
    );
}


//Autonomous Functions
void Drive::resetPosition(frc::Pose2d m_pose) {
    frc::Rotation2d angle{gyro.GetAngle()};
    m_poseEstimator.ResetPosition(angle, {
    frc::SwerveModulePosition{(units::meter_t)(m_flDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_frDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_blDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModulePosition{(units::meter_t)(m_brDriveMotor.GetEncoder().GetPosition() * M_PI * 0.1016 / 8.14),
     (units::radian_t)(m_brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)}
    }, m_pose);

    // double gyrovalues[6] = {m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(), 0.0, 0.0, 0.0, 0.0, 0.0};
    // ntinst.GetTable("limelight-primary")->PutNumber("imumode_set", 1);
    // ntinst.GetTable("limelight-primary")->PutNumberArray("robot_orientation_set", gyrovalues);
    // ntinst.GetTable("limelight-primary")->PutNumber("imumode_set", 3);
}

void Drive::autoDrive(
    frc::ChassisSpeeds autospeeds
) {
            units::meters_per_second_t autoxspeed = autospeeds.vx;
            units::meters_per_second_t autoyspeed = autospeeds.vy;
            units::radians_per_second_t autorotspeed = autospeeds.omega;
            SwerveDrive(autoxspeed, autoyspeed, autorotspeed, false);
}

frc::ChassisSpeeds Drive::getRobotRelativeChassisSpeeds() {
    return m_DriveKinematics.ToChassisSpeeds({
    frc::SwerveModuleState{(units::meters_per_second_t)(m_flDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60)),
     (units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModuleState{(units::meters_per_second_t)(m_frDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60)),
     (units::radian_t)(m_frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModuleState{(units::meters_per_second_t)(m_blDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60)),
     (units::radian_t)(m_blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)},
    frc::SwerveModuleState{(units::meters_per_second_t)(m_brDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60)),
     (units::radian_t)(m_brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)}
    });
}

frc2::CommandPtr Drive::DoNothingCommand() {
    return this->RunOnce(
        [this]() {
            pathplanner::PPHolonomicDriveController::overrideXYFeedback(
                []() {return 0_mps;},
                []() {return 0_mps;}
            );
            pathplanner::PPHolonomicDriveController::overrideRotationFeedback(
                []() {return 0_rad_per_s;}
            );
        }
    );
}

frc2::CommandPtr Drive::ClearOverridesCommand() {
    return this->RunOnce(
        [this]() {
            pathplanner::PPHolonomicDriveController::clearFeedbackOverrides();
        }
    );
}


//Drive Function for Swerve Drive
void Drive::SwerveDrive(
    units::meters_per_second_t xspeed,
    units::meters_per_second_t yspeed,
    units::radians_per_second_t rotspeed,
    bool fieldRelative
) {
    frc::ChassisSpeeds speeds{};
    if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed && fieldRelative) {
        speeds = {-xspeed, -yspeed, rotspeed};
    }
    else {
        speeds = {xspeed, yspeed, rotspeed};
    }
    frc::Rotation2d angle{};
    angle = m_poseEstimator.GetEstimatedPosition().Rotation();
    auto states = m_DriveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, angle) : speeds,
        0.02_s));
    m_DriveKinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);  
    auto [fl, fr, bl, br] = states;

    frc::Rotation2d flEncoderRotation{(units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)};
    frc::Rotation2d frEncoderRotation{(units::radian_t)(m_frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)};
    frc::Rotation2d blEncoderRotation{(units::radian_t)(m_blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)};
    frc::Rotation2d brEncoderRotation{(units::radian_t)(m_brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2)};

    fl.Optimize(flEncoderRotation);
    fr.Optimize(frEncoderRotation);
    bl.Optimize(blEncoderRotation);
    br.Optimize(brEncoderRotation);

    fl.CosineScale(flEncoderRotation);
    fr.CosineScale(frEncoderRotation);
    bl.CosineScale(blEncoderRotation);
    br.CosineScale(brEncoderRotation);

    //Drive Control
    units::volt_t flDrivePIDVoltage = (units::volt_t)(m_flDrivePID.Calculate(m_flDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60), fl.speed.value()));
    units::volt_t frDrivePIDVoltage = (units::volt_t)(m_frDrivePID.Calculate(m_frDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60), fr.speed.value()));
    units::volt_t blDrivePIDVoltage = (units::volt_t)(m_blDrivePID.Calculate(m_blDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60), bl.speed.value()));
    units::volt_t brDrivePIDVoltage = (units::volt_t)(m_brDrivePID.Calculate(m_brDriveMotor.GetEncoder().GetVelocity() * M_PI * 0.1016 / (8.14 * 60), br.speed.value()));

    units::volt_t flDriveFFVoltage = m_flDriveFF.Calculate(fl.speed);
    units::volt_t frDriveFFVoltage = m_frDriveFF.Calculate(fr.speed);
    units::volt_t blDriveFFVoltage = m_blDriveFF.Calculate(bl.speed);
    units::volt_t brDriveFFVoltage = m_brDriveFF.Calculate(br.speed);

    m_flDriveMotor.SetVoltage(flDrivePIDVoltage + flDriveFFVoltage);
    m_frDriveMotor.SetVoltage(frDrivePIDVoltage + frDriveFFVoltage);
    m_blDriveMotor.SetVoltage(blDrivePIDVoltage + blDriveFFVoltage);
    m_brDriveMotor.SetVoltage(brDrivePIDVoltage + brDriveFFVoltage);

    //Rotation Control
    units::volt_t flRotPIDVoltage = (units::volt_t)(m_flRotPID.Calculate((units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2), fl.angle.Radians()));
    units::volt_t frRotPIDVoltage = (units::volt_t)(m_frRotPID.Calculate((units::radian_t)(m_frRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2), fr.angle.Radians()));
    units::volt_t blRotPIDVoltage = (units::volt_t)(m_blRotPID.Calculate((units::radian_t)(m_blRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2), bl.angle.Radians()));
    units::volt_t brRotPIDVoltage = (units::volt_t)(m_brRotPID.Calculate((units::radian_t)(m_brRotEncoder.GetAbsolutePosition().GetValueAsDouble() * M_PI * 2), br.angle.Radians()));

    units::volt_t flRotFFVoltage = m_flRotFF.Calculate(m_flRotPID.GetSetpoint().velocity);
    units::volt_t frRotFFVoltage = m_frRotFF.Calculate(m_frRotPID.GetSetpoint().velocity);
    units::volt_t blRotFFVoltage = m_blRotFF.Calculate(m_blRotPID.GetSetpoint().velocity);
    units::volt_t brRotFFVoltage = m_brRotFF.Calculate(m_brRotPID.GetSetpoint().velocity);

    m_flRotMotor.SetVoltage(flRotPIDVoltage + flRotFFVoltage);
    m_frRotMotor.SetVoltage(frRotPIDVoltage + frRotFFVoltage);
    m_blRotMotor.SetVoltage(blRotPIDVoltage + blRotFFVoltage);
    m_brRotMotor.SetVoltage(brRotPIDVoltage + brRotFFVoltage);
}