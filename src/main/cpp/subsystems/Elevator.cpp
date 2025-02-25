// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() {
    m_elevatorMotor.GetEncoder().SetPosition(0.0);
    frc::SmartDashboard::PutData("ElevatorPID", &m_elevatorPID);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
    frc::SmartDashboard::PutNumber("Elevator Position", m_elevatorMotor.GetEncoder().GetPosition());

    // units::volt_t elevatorPIDVoltage = (units::volt_t)(m_elevatorPID.Calculate((units::meter_t)(m_coralMotor.GetEncoder().GetPosition()), elevatorPosition));
    // units::volt_t elevatorFFVoltage = m_elevatorFF.Calculate(m_elevatorPID.GetSetpoint().velocity);
    // m_elevatorMotor.SetVoltage(elevatorPIDVoltage + elevatorFFVoltage);
}

frc2::CommandPtr Elevator::raiseElevatorCommand() {
    return this->RunEnd(
        [this]() {m_elevatorMotor.Set(0.5);},
        [this]() {m_elevatorMotor.Set(0.0);}
    );
}

frc2::CommandPtr Elevator::lowerElevatorCommand() {
    return this->RunEnd(
        [this]() {m_elevatorMotor.Set(-0.5);},
        [this]() {m_elevatorMotor.Set(0.0);}
    );
}

frc2::CommandPtr Elevator::firstPositionCommand() {
    return this->RunOnce(
        [this]() {elevatorPosition = 0.0_m;}
    );
}

frc2::CommandPtr Elevator::secondPositionCommand() {
    return this->RunOnce(
        [this]() {elevatorPosition = 33.0_m;}
    );
}

frc2::CommandPtr Elevator::thirdPositionCommand() {
    return this->RunOnce(
        [this]() {elevatorPosition = 60.0_m;}
    );
}

frc2::CommandPtr Elevator::fourthPositionCommand() {
    return this->RunOnce(
        [this]() {elevatorPosition = 100.0_m;}
    );
}

frc2::CommandPtr Elevator::coralOutCommand() {
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce(
            [this]() {m_coralMotor.Set(0.5);}
        ),
        frc2::cmd::Run(
            [this]() {}
        ).FinallyDo(
            [this]() {m_coralMotor.Set(0.0);}
        )
    );
}

frc2::CommandPtr Elevator::coralInCommand() {
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce(
            [this]() {m_coralMotor.Set(-1.0);}
        ),
        frc2::cmd::Run(
            [this]() {}
        ).FinallyDo(
            [this]() {m_coralMotor.Set(0.0);}
        )
    );
}