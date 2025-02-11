// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() {
    m_elevatorMotor.GetEncoder().SetPosition(0.0);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
    frc::SmartDashboard::PutNumber("Elevator Position", m_elevatorMotor.GetEncoder().GetPosition());
}

frc2::CommandPtr Elevator::raiseElevatorCommand() {
    return this->RunEnd(
        [this]() {m_elevatorMotor.Set(1.0);},
        [this]() {m_elevatorMotor.Set(0.0);}
    );
}

frc2::CommandPtr Elevator::lowerElevatorCommand() {
    return this->RunEnd(
        [this]() {m_elevatorMotor.Set(-1.0);},
        [this]() {m_elevatorMotor.Set(0.0);}
    );
}