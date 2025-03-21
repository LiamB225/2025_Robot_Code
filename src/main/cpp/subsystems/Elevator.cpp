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
    frc::SmartDashboard::PutNumber("Elevator Goal", elevatorPosition.value());

    units::volt_t elevatorPIDVoltage = (units::volt_t)(m_elevatorPID.Calculate((units::meter_t)(m_elevatorMotor.GetEncoder().GetPosition()), elevatorPosition));
    units::volt_t elevatorFFVoltage = m_elevatorFF.Calculate(m_elevatorPID.GetSetpoint().velocity);
    frc::SmartDashboard::PutNumber("elevator FF", elevatorFFVoltage.value());
    frc::SmartDashboard::PutNumber("elevator PID", elevatorPIDVoltage.value());
    m_elevatorMotor.SetVoltage(elevatorPIDVoltage + elevatorFFVoltage);

    frc::SmartDashboard::PutBoolean("Barrier", m_coralBarrier.Get());
    frc::SmartDashboard::PutBoolean("Sensor", m_coralSensor.Get());
}

//Elevator Commands
frc2::CommandPtr Elevator::raiseElevatorCommand() {
    return this->Run(
        [this]() {elevatorPosition += 0.2_m;}
    );
}

frc2::CommandPtr Elevator::lowerElevatorCommand() {
    return this->Run(
        [this]() {elevatorPosition -= 0.2_m;}
    );
}

frc2::CommandPtr Elevator::firstPositionCommand() {
    return this->RunOnce(
        [this]() {elevatorPosition = 0.0_m;}
    );
}

frc2::CommandPtr Elevator::secondPositionCommand() {
    return this->RunOnce(
        [this]() {elevatorPosition = 50.3_m;}
    );
}

frc2::CommandPtr Elevator::thirdPositionCommand() {
    return this->RunOnce(
        [this]() {elevatorPosition = 70.8_m;}
    );
}

frc2::CommandPtr Elevator::fourthPositionCommand() {
    return this->RunOnce(
        [this]() {elevatorPosition = 100.0_m;}
    );
}


frc2::CommandPtr Elevator::coralOutCommand() {
    return this->StartEnd(
        [this]() {m_coralMotor.Set(0.5);},
        [this]() {m_coralMotor.Set(0.0);}
    );
}

frc2::CommandPtr Elevator::coralInCommand() {
    return this->StartEnd(
        [this]() {m_coralMotor.Set(-0.5);},
        [this]() {m_coralMotor.Set(0.0);}
    );
}

frc2::CommandPtr Elevator::shootCoralCommand() {
    return this->StartEnd(
        [this]() {m_coralMotor.Set(1.0);},
        [this]() {m_coralMotor.Set(0.0);}
    );
}

frc2::CommandPtr Elevator::automaticCoralGrabCommand() {
    return this->Run(
        [this]() {
            if(!m_coralBarrier.Get()) {
                m_coralMotor.Set(0.25);
            }
            if(!m_coralSensor.Get()) {
                m_coralMotor.Set(0.0);
            }
        }
    );
}