// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include "SwerveModule.h"


/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
 public:
    Drivetrain(AHRS& navx){
        ahrs = &navx;
        ahrs->Reset();
    }

    void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative);
    void UpdateOdometry();

    static constexpr units::meters_per_second_t kMaxSpeed = 1.0_mps;  // 1 meter per second

 private:
    //set module positions
    frc::Translation2d m_frontLeftLocation{+0.381_m, +0.381_m};
    frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
    frc::Translation2d m_backLeftLocation{-0.381_m, +0.381_m};
    frc::Translation2d m_backRightLocation{-0.381_m, -0.381_m};

    SwerveModule m_frontLeft{1, 2, 3, *ahrs};
    SwerveModule m_frontRight{1, 2, 3, *ahrs};
    SwerveModule m_backLeft{1, 2, 3, *ahrs};
    SwerveModule m_backRight{1, 2, 3, *ahrs};

    AHRS *ahrs;

    frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};

    frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, ahrs->GetRotation2d(), {m_frontLeft.GetPosition(m_frontLeft.getDrivePOS()), m_frontRight.GetPosition(m_frontRight.getDrivePOS()), m_backLeft.GetPosition(m_backLeft.getDrivePOS()), m_backRight.GetPosition(m_backRight.getDrivePOS())}};
};