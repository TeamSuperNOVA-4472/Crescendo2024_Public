// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <array>
#include <numbers>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

namespace constants {
namespace drive {
inline constexpr int kFLDrive = 1;
inline constexpr int kFLTurn = 2;
inline constexpr int kFRDrive = 3;
inline constexpr int kFRTurn = 4;
inline constexpr int kRLDrive = 5;
inline constexpr int kRLTurn = 6;
inline constexpr int kRRDrive = 7;
inline constexpr int kRRTurn = 8;

inline constexpr double kPositionConversionFactor = 0.047853945068174364;
inline constexpr double kVelocityConversionFactor = kPositionConversionFactor / 60;
inline constexpr double kAngleConversionFactor = 6.0704141305645065043561284162788;
inline constexpr double kAngleP = 0.01;
inline constexpr double kAngleI = 0;
inline constexpr double kAngleD = 0.00001;

inline constexpr std::array<int, 2> kLeftEncoderPorts = {0, 1};
inline constexpr std::array<int, 2> kRightEncoderPorts = {2, 3};
inline constexpr bool kLeftEncoderReversed = false;
inline constexpr bool kRightEncoderReversed = true;

inline constexpr int kEncoderCpr = 1024;
inline constexpr units::meter_t kWheelDiameter = 6_in;
inline constexpr units::meter_t kEncoderDistancePerPulse =
    (kWheelDiameter * std::numbers::pi) / static_cast<double>(kEncoderCpr);
}  // namespace drive

namespace shooter {

using kv_unit =
    units::compound_unit<units::compound_unit<units::volts, units::seconds>,
                         units::inverse<units::turns>>;
using kv_unit_t = units::unit_t<kv_unit>;

inline constexpr std::array<int, 2> kEncoderPorts = {4, 5};
inline constexpr bool kLeftEncoderReversed = false;
inline constexpr int kEncoderCpr = 1024;
inline constexpr units::turn_t kEncoderDistancePerPulse =
    units::turn_t{1.0} / static_cast<double>(kEncoderCpr);

inline constexpr int kShooterMotorPort = 4;
inline constexpr int kFeederMotorPort = 5;

inline constexpr units::turns_per_second_t kShooterFreeSpeed = 5300_tps;
inline constexpr units::turns_per_second_t kShooterTargetSpeed = 4000_tps;
inline constexpr units::turns_per_second_t kShooterTolerance = 50_tps;

inline constexpr double kP = 1.0;

inline constexpr units::volt_t kS = 0.05_V;
inline constexpr kv_unit_t kV = (12_V) / kShooterFreeSpeed;

inline constexpr double kFeederSpeed = 0.5;
}  // namespace shooter

namespace intake {
inline constexpr int kMotorPort = 6;
inline constexpr std::array<int, 2> kSolenoidPorts = {2, 3};
}  // namespace intake

namespace storage {
inline constexpr int kMotorPort = 7;
inline constexpr int kBallSensorPort = 6;
}  // namespace storage

namespace autonomous {
inline constexpr units::second_t kTimeout = 3_s;
inline constexpr units::meter_t kDriveDistance = 2_m;
inline constexpr double kDriveSpeed = 0.5;
}  // namespace autonomous

namespace oi {
inline constexpr int kDriverControllerPort = 0;
}  // namespace oi
}  // namespace constants
