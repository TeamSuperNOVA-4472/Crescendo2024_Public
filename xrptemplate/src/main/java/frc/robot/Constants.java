// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  public static final double kCountsPerMotorShaftRev = 12.0;
  public static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  public static final double kWheelDiameterInch = 2.3622; // 60 mm
  public static final double kMetersPerInch = 0.0254;
  public static final double kWheelDiameterMeter = kWheelDiameterInch * kMetersPerInch;
  public static final double kDriveBaseWidthInches = 6.102;
  public static final double kDriveBaseWidthMeters = kDriveBaseWidthInches * kMetersPerInch;
  public static final double kMaxSpeedInches = 26;
  public static final double kMaxSpeedMeters =  kMaxSpeedInches * kMetersPerInch;
  public static final double kPeriodicHz = 50;
  public static final double kGyroDrift = -0.012;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
