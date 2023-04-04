// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"


void Drivetrain::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative) {

  auto states = m_kinematics.ToSwerveModuleStates(fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs->GetRotation2d()): frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
    m_odometry.Update(ahrs->GetRotation2d(), {m_frontLeft.GetPosition(m_frontLeft.getDrivePOS()), m_frontRight.GetPosition(m_frontRight.getDrivePOS()), m_backLeft.GetPosition(m_backLeft.getDrivePOS()), m_backRight.GetPosition(m_backRight.getDrivePOS())});
}
