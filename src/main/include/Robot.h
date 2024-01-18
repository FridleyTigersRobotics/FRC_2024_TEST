// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/AnalogEncoder.h>
#include <rev/cansparkmax.h>
#include <frc/XboxController.h>

#include "SwerveModule.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::XboxController m_controller{0};


  //frc::AnalogEncoder m_analog0{0};
  //frc::AnalogEncoder m_analog1{1};
  //frc::AnalogEncoder m_analog2{2};
  //frc::AnalogEncoder m_analog3{3};

 // rev::CANSparkMax m_driveMotor0{11,rev::CANSparkLowLevel::MotorType::kBrushless};
 // rev::CANSparkMax m_driveMotor1{12,rev::CANSparkLowLevel::MotorType::kBrushless};
 // rev::CANSparkMax m_driveMotor2{14,rev::CANSparkLowLevel::MotorType::kBrushless};
 // rev::CANSparkMax m_driveMotor3{16,rev::CANSparkLowLevel::MotorType::kBrushless};

  //rev::CANSparkMax m_turnMotor0{10,rev::CANSparkLowLevel::MotorType::kBrushless};
 // rev::CANSparkMax m_turnMotor1{13,rev::CANSparkLowLevel::MotorType::kBrushless};
 // rev::CANSparkMax m_turnMotor2{15,rev::CANSparkLowLevel::MotorType::kBrushless};
  //rev::CANSparkMax m_turnMotor3{17,rev::CANSparkLowLevel::MotorType::kBrushless};

SwerveModule m_frontLeft {10, 11, 0};
SwerveModule m_frontRight{12, 13, 1};
SwerveModule m_backLeft  {14, 15, 2};
SwerveModule m_backRight {16, 17, 3};

  //rev::SparkRelativeEncoder m_driveEncoder0 = m_driveMotor0.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  //rev::SparkRelativeEncoder m_driveEncoder1 = m_driveMotor1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  //rev::SparkRelativeEncoder m_driveEncoder2 = m_driveMotor2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  //rev::SparkRelativeEncoder m_driveEncoder3 = m_driveMotor3.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
