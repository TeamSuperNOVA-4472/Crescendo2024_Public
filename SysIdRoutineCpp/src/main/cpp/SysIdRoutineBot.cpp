// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SysIdRoutineBot.h"

#include <frc2/command/Commands.h>
#include <frc/MathUtil.h>

SysIdRoutineBot::SysIdRoutineBot() {
  ConfigureBindings();
}

void SysIdRoutineBot::ConfigureBindings() {
  m_drive.SetDefaultCommand(m_drive.ArcadeDriveCommand(
      [this] { return frc::ApplyDeadband(-m_driverController.GetLeftY(), 0.1); },
      [this] { return -m_driverController.GetRightX(); }));

  m_driverController.A().WhileTrue(
      m_drive.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  m_driverController.B().WhileTrue(
      m_drive.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  m_driverController.X().WhileTrue(
      m_drive.SysIdDynamic(frc2::sysid::Direction::kForward));
  m_driverController.Y().WhileTrue(
      m_drive.SysIdDynamic(frc2::sysid::Direction::kReverse));
}

frc2::CommandPtr SysIdRoutineBot::GetAutonomousCommand() {
  return m_drive.Run([] {});
}
