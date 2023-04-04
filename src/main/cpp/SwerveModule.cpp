// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>


SwerveModule::SwerveModule(const int driveMotorID, const int turningMotorID,const int turningEncoderID): m_driveMotor(driveMotorID), m_turningMotor(turningMotorID), m_turningEncoder(turningEncoderID) {}

frc::SwerveModulePosition SwerveModule::GetPosition(double distanceDrive) const{
    return {units::meter_t{distanceDrive}, {ahrs->GetRotation2d()}};
}

double SwerveModule::getDrivePOS(){
    return ((m_driveMotor.GetSelectedSensorPosition())*(5)); //sensor units multiplied by meters per sensor unit to get distance in meters
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(referenceState, {ahrs->GetRotation2d()});

  double angle = (((double) state.angle.Degrees())*(4096.0/ 360.0));

  // Set the motor outputs.
  m_driveMotor.Set((double) state.speed);
  m_turningMotor.Set(TalonFXControlMode::Position, angle);
}
