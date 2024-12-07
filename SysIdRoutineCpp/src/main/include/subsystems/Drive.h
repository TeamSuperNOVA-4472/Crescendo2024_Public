// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>

#include <frc/Encoder.h>
#include <frc/RobotController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "Constants.h"

class Drive : public frc2::SubsystemBase {
 public:
  Drive();

  frc2::CommandPtr ArcadeDriveCommand(std::function<double()> fwd,
                                      std::function<double()> rot);
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

 private:
  rev::CANSparkMax m_FLDrive{constants::drive::kFLDrive, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_FLTurn{constants::drive::kFLTurn, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_FRDrive{constants::drive::kFRDrive, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_FRTurn{constants::drive::kFRTurn, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_RLDrive{constants::drive::kRLDrive, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_RLTurn{constants::drive::kRLTurn, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_RRDrive{constants::drive::kRRDrive, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_RRTurn{constants::drive::kRRTurn, rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkRelativeEncoder m_FLEncoder = m_FLDrive.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_FREncoder = m_FRDrive.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_RLEncoder = m_RLDrive.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_RREncoder = m_RRDrive.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  rev::SparkRelativeEncoder m_FLTurnEncoder = m_FLTurn.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_FRTurnEncoder = m_FRTurn.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_RLTurnEncoder = m_RLTurn.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
  rev::SparkRelativeEncoder m_RRTurnEncoder = m_RRTurn.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

  rev::SparkPIDController m_FLTurnController = m_FLTurn.GetPIDController();
  rev::SparkPIDController m_FRTurnController = m_FRTurn.GetPIDController();
  rev::SparkPIDController m_RLTurnController = m_RLTurn.GetPIDController();
  rev::SparkPIDController m_RRTurnController = m_RRTurn.GetPIDController();

  frc2::sysid::SysIdRoutine m_sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
            m_FLDrive.SetVoltage(driveVoltage);
            m_FRDrive.SetVoltage(driveVoltage);
            m_RLDrive.SetVoltage(driveVoltage);
            m_RRDrive.SetVoltage(driveVoltage);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("drive-FL")
                .voltage(m_FLDrive.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{m_FLEncoder.GetPosition()})
                .velocity(units::meters_per_second_t{m_FLEncoder.GetVelocity()});
            log->Motor("drive-FR")
                .voltage(m_FRDrive.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{m_FREncoder.GetPosition()})
                .velocity(units::meters_per_second_t{m_FREncoder.GetVelocity()});
            log->Motor("drive-RL")
                .voltage(m_RLDrive.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{m_RLEncoder.GetPosition()})
                .velocity(units::meters_per_second_t{m_RLEncoder.GetVelocity()});
            log->Motor("drive-RR")
                .voltage(m_RRDrive.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(units::meter_t{m_RREncoder.GetPosition()})
                .velocity(units::meters_per_second_t{m_RREncoder.GetVelocity()});
          },
          this}};
};
