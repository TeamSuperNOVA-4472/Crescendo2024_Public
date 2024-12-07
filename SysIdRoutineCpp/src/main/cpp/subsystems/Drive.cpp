// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

#include <frc2/command/Commands.h>

Drive::Drive() {
  m_FLEncoder.SetPositionConversionFactor(constants::drive::kPositionConversionFactor);
  m_FLEncoder.SetVelocityConversionFactor(constants::drive::kVelocityConversionFactor);
  m_FREncoder.SetPositionConversionFactor(constants::drive::kPositionConversionFactor);
  m_FREncoder.SetVelocityConversionFactor(constants::drive::kVelocityConversionFactor);
  m_RLEncoder.SetPositionConversionFactor(constants::drive::kPositionConversionFactor);
  m_RLEncoder.SetVelocityConversionFactor(constants::drive::kVelocityConversionFactor);
  m_RREncoder.SetPositionConversionFactor(constants::drive::kPositionConversionFactor);
  m_RREncoder.SetVelocityConversionFactor(constants::drive::kVelocityConversionFactor);

  m_FLTurnEncoder.SetPositionConversionFactor(constants::drive::kAngleConversionFactor);
  m_FLTurnEncoder.SetPosition(0);
  m_FLTurnController.SetP(constants::drive::kAngleP);
  m_FLTurnController.SetI(constants::drive::kAngleI);
  m_FLTurnController.SetD(constants::drive::kAngleD);
  m_FLTurnController.SetReference(0, rev::ControlType::kPosition);

  m_FRTurnEncoder.SetPositionConversionFactor(constants::drive::kAngleConversionFactor);
  m_FRTurnEncoder.SetPosition(0);
  m_FRTurnController.SetP(constants::drive::kAngleP);
  m_FRTurnController.SetI(constants::drive::kAngleI);
  m_FRTurnController.SetD(constants::drive::kAngleD);
  m_FRTurnController.SetReference(0, rev::ControlType::kPosition);

  m_RLTurnEncoder.SetPositionConversionFactor(constants::drive::kAngleConversionFactor);
  m_RLTurnEncoder.SetPosition(0);
  m_RLTurnController.SetP(constants::drive::kAngleP);
  m_RLTurnController.SetI(constants::drive::kAngleI);
  m_RLTurnController.SetD(constants::drive::kAngleD);
  m_RLTurnController.SetReference(0, rev::ControlType::kPosition);

  m_RRTurnEncoder.SetPositionConversionFactor(constants::drive::kAngleConversionFactor);
  m_RRTurnEncoder.SetPosition(0);
  m_RRTurnController.SetP(constants::drive::kAngleP);
  m_RRTurnController.SetI(constants::drive::kAngleI);
  m_RRTurnController.SetD(constants::drive::kAngleD);
  m_RRTurnController.SetReference(0, rev::ControlType::kPosition);

}

frc2::CommandPtr Drive::ArcadeDriveCommand(std::function<double()> fwd,
                                           std::function<double()> rot) {
  return frc2::cmd::Run([this, fwd, rot] {
      m_FLDrive.Set(fwd());
      m_FRDrive.Set(fwd());
      m_RLDrive.Set(fwd());
      m_RRDrive.Set(fwd());
    },
    {this})
      .WithName("Arcade Drive");
}

frc2::CommandPtr Drive::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Drive::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}
