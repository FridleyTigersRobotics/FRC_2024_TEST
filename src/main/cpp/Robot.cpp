// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {
  double kWheelRadius       = 1.0;
  double kEncoderResolution = 1.0;

  double positonConversionFactor  = 3.9*std::numbers::pi/8.14*0.0254; //* std::numbers::pi * kWheelRadius / kEncoderResolution;
  double velocityConversionFactor = (1.0/60.0) * positonConversionFactor;//2.0 * std::numbers::pi * kWheelRadius / kEncoderResolution;
  
  m_analog0.SetDistancePerRotation( 2.0 * std::numbers::pi ); //1.230863 Drive motor #10
  m_analog1.SetDistancePerRotation( 2.0 * std::numbers::pi ); //0.909437 Drive motor #12
  m_analog2.SetDistancePerRotation( 2.0 * std::numbers::pi ); //0.255626 Drive motor #14
  m_analog3.SetDistancePerRotation( 2.0 * std::numbers::pi ); //4.980153 Drive motor #16
  
  //
  //-----------|Front|------------
  //  16----------------------12
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  |------------------------|
  //  14----------------------10
  //------------|back|-------------
  //
  
  // output = X * positonConversionFactor
  // 0      = 0 * positonConversionFactor
  // 1      = 8.192 * positonConversionFactor
  // 1 / 8.192 = positonConversionFactor

// RPM = Revs / Minute
// MPS Meters / Second

// MPS = RPM * .03798903421 = 3.9*pi/8.192*0.0254

// Meters / Seconds = Revs / Minute * X

// X = (1.0/60.0) * 3.9*std::numbers::pi*0.0254


//10/8.1 revs/second
//meters per rev= 3.9*std::numbers::*0.0254




  m_driveEncoder0.SetPositionConversionFactor(positonConversionFactor);
  m_driveEncoder0.SetPosition(0);
  m_driveEncoder1.SetPositionConversionFactor(positonConversionFactor);
  m_driveEncoder1.SetPosition(0);
  m_driveEncoder2.SetPositionConversionFactor(positonConversionFactor);
  m_driveEncoder2.SetPosition(0);
  m_driveEncoder3.SetPositionConversionFactor(positonConversionFactor);
  m_driveEncoder3.SetPosition(0);
  m_driveEncoder0.SetVelocityConversionFactor(velocityConversionFactor);
  m_driveEncoder1.SetVelocityConversionFactor(velocityConversionFactor);
  m_driveEncoder2.SetVelocityConversionFactor(velocityConversionFactor);
  m_driveEncoder3.SetVelocityConversionFactor(velocityConversionFactor);

  /*if( m_controller.GetAButton() )
  {

  }*/




}

void Robot::TeleopPeriodic() {

  frc::SmartDashboard::PutNumber("Analog 0", m_analog0.GetDistance());
  frc::SmartDashboard::PutNumber("Analog 1", m_analog1.GetDistance());
  frc::SmartDashboard::PutNumber("Analog 2", m_analog2.GetDistance()); 
  frc::SmartDashboard::PutNumber("Analog 3", m_analog3.GetDistance());

  frc::SmartDashboard::PutNumber("POS Drive Enc 0", m_driveEncoder0.GetPosition());
  frc::SmartDashboard::PutNumber("POS Drive Enc 1", m_driveEncoder1.GetPosition());
  frc::SmartDashboard::PutNumber("POS Drive Enc 2", m_driveEncoder2.GetPosition()); 
  frc::SmartDashboard::PutNumber("POS Drive Enc 3", m_driveEncoder3.GetPosition());

  frc::SmartDashboard::PutNumber("VEL Drive Enc 0", m_driveEncoder0.GetVelocity());
  frc::SmartDashboard::PutNumber("VEL Drive Enc 1", m_driveEncoder1.GetVelocity());
  frc::SmartDashboard::PutNumber("VEL Drive Enc 2", m_driveEncoder2.GetVelocity()); 
  frc::SmartDashboard::PutNumber("VEL Drive Enc 3", m_driveEncoder3.GetVelocity());

  if( m_controller.GetAButton() )
  {
    m_driveMotor0.Set(0.3);
    m_driveMotor1.Set(0.3);
    m_driveMotor2.Set(0.3);
    m_driveMotor3.Set(0.3);
  }
  else if( m_controller.GetBButton() )
  {
    m_driveMotor0.Set(-0.3);
    m_driveMotor1.Set(-0.3);
    m_driveMotor2.Set(-0.3);
    m_driveMotor3.Set(-0.3);
  }
  else
  {
    m_driveMotor0.Set(0);
    m_driveMotor1.Set(0);
    m_driveMotor2.Set(0);
    m_driveMotor3.Set(0);
  }

if(m_controller.GetLeftBumper())
{
  m_turnMotor0.Set(0.3);
  m_turnMotor1.Set(0.3);
  m_turnMotor2.Set(0.3);
  m_turnMotor3.Set(0.3);
}
else if(m_controller.GetRightBumper())
{
  m_turnMotor0.Set(-0.3);
  m_turnMotor1.Set(-0.3);
  m_turnMotor2.Set(-0.3);
  m_turnMotor3.Set(-0.3);
}
else
{
  m_turnMotor0.Set(0);
  m_turnMotor1.Set(0);
  m_turnMotor2.Set(0);
  m_turnMotor3.Set(0);
}









}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
