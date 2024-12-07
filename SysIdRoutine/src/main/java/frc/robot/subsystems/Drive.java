// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drive extends SubsystemBase {

  private static final PositionVoltage REQUEST = new PositionVoltage(0).withSlot(0);

  private final TalonFX mFLDrive = new TalonFX(0, "canivore");  
  private final TalonFX mFLTurn = new TalonFX(1, "canivore");  
  private final TalonFX mFRDrive = new TalonFX(2, "canivore");   
  private final TalonFX mFRTurn = new TalonFX(3, "canivore"); 
  private final TalonFX mRLDrive = new TalonFX(4, "canivore");   
  private final TalonFX mRLTurn = new TalonFX(5, "canivore"); 
  private final TalonFX mRRDrive = new TalonFX(6, "canivore");  
  private final TalonFX  mRRTurn = new TalonFX(7, "canivore"); 


  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  // Create a new SysId routine for characterizing the drive.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motors.
              (Measure<Voltage> volts) -> {
                double battVolts = RobotController.getBatteryVoltage();
                mFLDrive.set(volts.in(Volts)/battVolts);
                mFRDrive.set(volts.in(Volts)/battVolts);
                mRLDrive.set(volts.in(Volts)/battVolts);
                mRRDrive.set(volts.in(Volts)/battVolts);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                logMotor(log, "FLDrive", mFLDrive, m_appliedVoltage, m_distance, m_velocity);
                logMotor(log, "FRDrive", mFLDrive, m_appliedVoltage, m_distance, m_velocity);
                logMotor(log, "RLDrive", mFLDrive, m_appliedVoltage, m_distance, m_velocity);
                logMotor(log, "RRDrive", mFLDrive, m_appliedVoltage, m_distance, m_velocity);
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("drive")
              this));

  /** Creates a new Drive subsystem. */
  public Drive() {

    // Sets the distance per pulse for the encoders
    setEncoder(mFLDrive);
    setEncoder(mFRDrive);
    setEncoder(mRLDrive);
    setEncoder(mRRDrive);
    initalizeAngleMotor(mFLTurn);
    initalizeAngleMotor(mFRTurn);
    initalizeAngleMotor(mRLTurn);
    initalizeAngleMotor(mRRTurn);
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> {
      double pctVal = fwd.getAsDouble();
      mFLDrive.set(pctVal);
      mFRDrive.set(pctVal);
      mRLDrive.set(pctVal);
      mRRDrive.set(pctVal);
    });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  private static void setEncoder(TalonFX pEncoder) {
    pEncoder.getConfigurator().apply(
      new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(1/DriveConstants.kPositionConversionFactor)
    );
    pEncoder.setPosition(0);
  }

  private static void logMotor(
    SysIdRoutineLog pLog,
    String pMotorName,
    TalonFX pMotor,
    MutableMeasure<Voltage> pAppliedVoltage,
    MutableMeasure<Distance> pDistance,
    MutableMeasure<Velocity<Distance>> pVelocity) {
    pLog.motor(pMotorName)
      .voltage(
          pAppliedVoltage.mut_replace(
              pMotor.get() * RobotController.getBatteryVoltage(), Volts))
      .linearPosition(pDistance.mut_replace(pMotor.getPosition().getValueAsDouble(), Meters))
      .linearVelocity(
          pVelocity.mut_replace(pMotor.getVelocity().getValueAsDouble(), MetersPerSecond));
  }

  private static void initalizeAngleMotor(TalonFX pTalonFx) {
    pTalonFx.getConfigurator().apply(
      new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        .withSensorToMechanismRatio(DriveConstants.kAngleConversionFactor)
    );
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = DriveConstants.kAngleP;
    slot0Configs.kI = DriveConstants.kAngleI;
    slot0Configs.kD = DriveConstants.kAngleD;
    pTalonFx.getConfigurator().apply(slot0Configs);
    pTalonFx.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FLDrive Pos", mFLDrive.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("FRDrive Pos", mFRDrive.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("RLDrive Pos", mRLDrive.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("RRDrive Pos", mRRDrive.getPosition().getValueAsDouble());
    mFLTurn.setControl(REQUEST.withPosition(0));
    mFRTurn.setControl(REQUEST.withPosition(0));
    mRLTurn.setControl(REQUEST.withPosition(0));
    mRRTurn.setControl(REQUEST.withPosition(0));
  }
}
