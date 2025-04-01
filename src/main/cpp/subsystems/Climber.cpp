// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber() {
    frc::SmartDashboard::PutData("ClimberPID", &m_climberPID);
}

// This method will be called once per scheduler run
void Climber::Periodic() {
    frc::SmartDashboard::PutNumber("ClimberPostition", m_climberMotor.GetEncoder().GetPosition());
}

frc2::CommandPtr Climber::ClimbCommand() {
    return this->Run(
        [this]() {
            units::volt_t PIDVoltage = (units::volt_t)(m_climberPID.Calculate(m_climberMotor.GetEncoder().GetPosition(), 125.0));
            m_climberMotor.SetVoltage(PIDVoltage);
        }
    );
}

frc2::CommandPtr Climber::ClimbOutCommand() {
    return this->Run(
        [this]() {
            units::volt_t PIDVoltage = (units::volt_t)(m_climberPID.Calculate(m_climberMotor.GetEncoder().GetPosition(), -80.0));
            m_climberMotor.SetVoltage(PIDVoltage);
        }
    );
}

frc2::CommandPtr Climber::ClimbIn() {
    return this->StartEnd(
        [this]() {
            m_climberMotor.Set(1.0);
        },
        [this]() {
            m_climberMotor.Set(0.0);
        }
    );
}

frc2::CommandPtr Climber::ClimbOut() {
    return this->StartEnd(
        [this]() {
            m_climberMotor.Set(-1.0);
        },
        [this]() {
            m_climberMotor.Set(0.0);
        }
    );
}