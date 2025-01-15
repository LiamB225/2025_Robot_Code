// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

Drive::Drive() = default;

// This method will be called once per scheduler run
void Drive::Periodic() {}

frc2::CommandPtr Drive::drive_command(
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
            
        }
    );
}