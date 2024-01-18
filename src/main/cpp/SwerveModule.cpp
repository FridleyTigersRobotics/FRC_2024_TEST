// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>
//#include <Cameron's Brain> [ERROR:NONEXISTANT]
#include <frc/geometry/Rotation2d.h>

SwerveModule::SwerveModule(const int driveMotorChannel,
                           const int turningMotorChannel,
                           const int turningEncoderChannel)
    : m_driveMotor(driveMotorChannel,rev::CANSparkLowLevel::MotorType::kBrushless),
      m_turningMotor(turningMotorChannel,rev::CANSparkLowLevel::MotorType::kBrushless),
      //Somebody once told me the world is going to roll me I aint the sharpest tool in the shed. She was looking kind of dumb with her finger nad her thumb in the shape of an L on her forehead

      
      m_turningEncoder(turningEncoderChannel) {
  // Set the distance per pulse for the drive encoder. We can simply use the
  // distance traveled for one rotation of the wheel divided by the encoder
  // resolution.
  double positonConversionFactor  = 3.9*std::numbers::pi/8.14*0.0254;
  double velocityConversionFactor = (1.0/60.0) * positonConversionFactor;

  m_driveEncoder.SetPositionConversionFactor(positonConversionFactor);
  m_driveEncoder.SetVelocityConversionFactor(velocityConversionFactor);

  // Set the distance (in this case, angle) per pulse for the turning encoder.
  // This is the the angle through an entire rotation (2 * std::numbers::pi)
  // divided by the encoder resolution.
  m_turningEncoder.SetDistancePerRotation( 2.0 * std::numbers::pi );
  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(
      -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return {units::meters_per_second_t{m_driveEncoder.GetVelocity()},
          units::radian_t{m_turningEncoder.GetAbsolutePosition()}};
}

frc::SwerveModulePosition SwerveModule::GetPosition() const {
  return {units::meter_t{m_driveEncoder.GetPosition()},
          units::radian_t{m_turningEncoder.GetAbsolutePosition()}};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState& referenceState) {
  frc::Rotation2d encoderRotation{
      units::radian_t{m_turningEncoder.GetAbsolutePosition()}};

  // Optimize the reference state to avoid spinning further than 90 degrees
  auto state =
      frc::SwerveModuleState::Optimize(referenceState, encoderRotation);

  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  state.speed *= (state.angle - encoderRotation).Cos();

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(
      m_driveEncoder.GetPositionConversionFactor(), state.speed.value());

  const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

  // Calculate the turning motor output from the turning PID controller.
  const auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t{m_turningEncoder.GetAbsolutePosition()}, state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  // Set the motor outputs.
  m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
  m_turningMotor.SetVoltage(units::volt_t{turnOutput} + turnFeedforward);
}