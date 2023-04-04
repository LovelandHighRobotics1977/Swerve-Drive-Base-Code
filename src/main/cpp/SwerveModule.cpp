// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>

#include <frc/geometry/Rotation2d.h>


SwerveModule::SwerveModule(const int driveMotorID, const int turningMotorID,const int turningEncoderID, AHRS& navx): m_driveMotor(driveMotorID), m_turningMotor(turningMotorID), m_turningEncoder(turningEncoderID) {
  	ahrs = &navx;
  	ahrs->Reset();
  
  	m_driveMotor.SetNeutralMode(NeutralMode::Brake);

	m_driveMotor.ConfigFactoryDefault();
	m_driveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 0);
  	m_driveMotor.Config_kP(0, 0.01);
  	m_driveMotor.Config_kI(0, 0);
  	m_driveMotor.Config_kD(0, 0.8);
  	m_driveMotor.Config_kF(0, 1);
	m_driveMotor.ConfigNominalOutputForward(0);
	m_driveMotor.ConfigNominalOutputReverse(0);
	m_driveMotor.ConfigPeakOutputForward(1);
	m_driveMotor.ConfigPeakOutputReverse(-1);

	m_turningMotor.SetSensorPhase(true);
	m_turningMotor.SetNeutralMode(NeutralMode::Brake);

	m_turningMotor.ConfigFactoryDefault();
	m_turningMotor.ConfigRemoteFeedbackFilter(turningMotorID, RemoteSensorSource(13), 0, 0);
	m_turningMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, 0, 0); // PIDLoop=0, timeoutMs=0
	m_turningMotor.Config_kP(0, 1.7);
	m_turningMotor.Config_kI(0, .0016);
	m_turningMotor.Config_kD(0, 160);
	m_turningMotor.Config_kF(0, 0);
	m_turningMotor.Config_IntegralZone(0, 20);
	m_turningMotor.ConfigNominalOutputForward(0);
	m_turningMotor.ConfigNominalOutputReverse(0);
	m_turningMotor.ConfigPeakOutputForward(1);
	m_turningMotor.ConfigPeakOutputReverse(-1);

	switch (turningMotorID){
	case 2: //front left
		m_turningEncoder.ConfigMagnetOffset(105);
		break;
	case 5: //front right
		m_turningEncoder.ConfigMagnetOffset(110);
		break;
	case 8: //rear left
		m_turningEncoder.ConfigMagnetOffset(250);
		break;
	case 11: //rear right
		m_turningEncoder.ConfigMagnetOffset(148);
		break;
	}
	m_turningEncoder.SetPositionToAbsolute();
}

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