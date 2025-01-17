// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

Drive::Drive() {
    m_flRotPID.EnableContinuousInput(-(units::radian_t)(M_PI), (units::radian_t)(M_PI));
    m_frRotPID.EnableContinuousInput(-(units::radian_t)(M_PI), (units::radian_t)(M_PI));
    m_blRotPID.EnableContinuousInput(-(units::radian_t)(M_PI), (units::radian_t)(M_PI));
    m_brRotPID.EnableContinuousInput(-(units::radian_t)(M_PI), (units::radian_t)(M_PI));
}

// This method will be called once per scheduler run
void Drive::Periodic() {}

frc2::CommandPtr Drive::driveCommand(
    std::function<double(void)> drive_power,
    std::function<double(void)> strafe_power,
    std::function<double(void)> rot_power
) {
    return this->Run(
        [this, drive_power, strafe_power, rot_power]() {
            units::meters_per_second_t drive_vel = drive_power() * kMaxSpeed;
            units::meters_per_second_t strafe_vel = strafe_power() * kMaxSpeed;
            units::radians_per_second_t rot_vel = rot_power() * kRotMaxSpeed;

            frc::ChassisSpeeds speeds{drive_vel, strafe_vel, rot_vel};
            auto states = m_DriveKinematics.ToSwerveModuleStates(speeds);
            m_DriveKinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);
            auto [fl, fr, bl, br] = states;
            
            frc::Rotation2d flEncoderRotation{(units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble()) * M_PI};
            frc::Rotation2d frEncoderRotation{(units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble()) * M_PI};
            frc::Rotation2d blEncoderRotation{(units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble()) * M_PI};
            frc::Rotation2d brEncoderRotation{(units::radian_t)(m_flRotEncoder.GetAbsolutePosition().GetValueAsDouble()) * M_PI};

            fl.Optimize(flEncoderRotation);
            fr.Optimize(frEncoderRotation);
            bl.Optimize(blEncoderRotation);
            br.Optimize(brEncoderRotation);

            fl.CosineScale(flEncoderRotation);
            fr.CosineScale(frEncoderRotation);
            bl.CosineScale(blEncoderRotation);
            br.CosineScale(brEncoderRotation);

            units::volt_t flDrivePIDVoltage = (units::volt_t)(m_flDrivePID.Calculate(m_flDriveEncoder.GetVelocity() * M_PI * 0.1016 / 5.14, fl.speed.value()));
            units::volt_t frDrivePIDVoltage = (units::volt_t)(m_frDrivePID.Calculate(m_frDriveEncoder.GetVelocity() * M_PI * 0.1016 / 5.14, fr.speed.value()));
            units::volt_t blDrivePIDVoltage = (units::volt_t)(m_blDrivePID.Calculate(m_blDriveEncoder.GetVelocity() * M_PI * 0.1016 / 5.14, bl.speed.value()));
            units::volt_t brDrivePIDVoltage = (units::volt_t)(m_brDrivePID.Calculate(m_brDriveEncoder.GetVelocity() * M_PI * 0.1016 / 5.14, br.speed.value()));

            units::volt_t flDriveFFVoltage = m_flDriveFF.Calculate(fl.speed);
            units::volt_t frDriveFFVoltage = m_flDriveFF.Calculate(fr.speed);
            units::volt_t blDriveFFVoltage = m_flDriveFF.Calculate(bl.speed);
            units::volt_t brDriveFFVoltage = m_flDriveFF.Calculate(br.speed);

            m_flDriveMotor.SetVoltage(flDrivePIDVoltage + flDriveFFVoltage);
            m_frDriveMotor.SetVoltage(frDrivePIDVoltage + frDriveFFVoltage);
            m_blDriveMotor.SetVoltage(blDrivePIDVoltage + blDriveFFVoltage);
            m_brDriveMotor.SetVoltage(brDrivePIDVoltage + brDriveFFVoltage);

            units::volt_t flRotPIDVoltage = (units::volt_t)(m_flRotPID.Calculate((units::radian_t)(m_flRotEncoder.GetPosition().GetValueAsDouble() * M_PI), fl.angle.Radians()));
            units::volt_t frRotPIDVoltage = (units::volt_t)(m_frRotPID.Calculate((units::radian_t)(m_frRotEncoder.GetPosition().GetValueAsDouble() * M_PI), fr.angle.Radians()));
            units::volt_t blRotPIDVoltage = (units::volt_t)(m_blRotPID.Calculate((units::radian_t)(m_blRotEncoder.GetPosition().GetValueAsDouble() * M_PI), bl.angle.Radians()));
            units::volt_t brRotPIDVoltage = (units::volt_t)(m_brRotPID.Calculate((units::radian_t)(m_brRotEncoder.GetPosition().GetValueAsDouble() * M_PI), br.angle.Radians()));

            units::volt_t flRotFFVoltage = m_flRotFF.Calculate(m_flRotPID.GetSetpoint().velocity);
            units::volt_t frRotFFVoltage = m_frRotFF.Calculate(m_frRotPID.GetSetpoint().velocity);
            units::volt_t blRotFFVoltage = m_blRotFF.Calculate(m_blRotPID.GetSetpoint().velocity);
            units::volt_t brRotFFVoltage = m_brRotFF.Calculate(m_brRotPID.GetSetpoint().velocity);

            m_flRotMotor.SetVoltage(flRotPIDVoltage + flRotFFVoltage);
            m_frRotMotor.SetVoltage(frRotPIDVoltage + frRotFFVoltage);
            m_blRotMotor.SetVoltage(blRotPIDVoltage + blRotFFVoltage);
            m_brRotMotor.SetVoltage(brRotPIDVoltage + brRotFFVoltage);
        }
    );
}