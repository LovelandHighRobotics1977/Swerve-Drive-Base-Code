// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "units/length.h"
#include "units/angle.h"
#include <ctre/Phoenix.h>
#include "AHRS.h"

class SwerveModule {
 public:
    SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID, AHRS& navx);
    frc::SwerveModulePosition GetPosition(double distanceDrive) const;
    void SetDesiredState(const frc::SwerveModuleState& state);
    double getDrivePOS();

 private:
    WPI_TalonFX m_driveMotor;
    WPI_TalonFX m_turningMotor;
    WPI_CANCoder m_turningEncoder;
    AHRS *ahrs;
};