// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Drivetrain extends SubsystemBase {

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  private final Timer mTimer = new Timer();

  private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(kDriveBaseWidthMeters);
  private final DifferentialDriveOdometry mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(kCountsPerMotorShaftRev), 
    getLeftDistanceMeters(), getRightDistanceMeters());

  PIDController m_speedController = new PIDController(0.05, 0, 0);
  SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.125 / kMaxSpeedMeters, 0.875 / kMaxSpeedMeters);

  private double m_prevLeftDist = 0;
  private double m_prevRightDist = 0; 
  private double m_LeftSpeed = 0;
  private double m_RightSpeed = 0; 
  LinearFilter m_LeftSpeedAvg = LinearFilter.movingAverage(5);
  LinearFilter m_RightSpeedAvg = LinearFilter.movingAverage(5);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterMeter) / kCountsPerRevolution);

    mTimer.start();

    AutoBuilder.configureRamsete(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentSpeeds, // Current ChassisSpeeds supplier
      this::drive, // Method that will drive the robot given ChassisSpeeds
      new ReplanningConfig(), // Default path replanning config. See the API for the options here
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
    resetPose(new Pose2d());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SmartDashboard.putNumber("Target X Speed (m/s)", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Target Y (m/s)", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Target Turn Speed (rad/s)", chassisSpeeds.omegaRadiansPerSecond);
    DifferentialDriveWheelSpeeds wheelSpeeds = mKinematics.toWheelSpeeds(chassisSpeeds);
    drive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    WheelSpeeds wheelOutput = DifferentialDrive.arcadeDriveIK(xaxisSpeed, zaxisRotate, false);
    double targetLeftSpeed = wheelOutput.left * kMaxSpeedMeters;
    double targetRightSpeed = wheelOutput.right * kMaxSpeedMeters;
    drive(targetLeftSpeed, targetRightSpeed);
  }

  public void drive(double leftSpeedMetersPerSec, double rightSpeedMetersPerSec) {
    SmartDashboard.putNumber("Target Left Speed (m/s)", leftSpeedMetersPerSec);
    SmartDashboard.putNumber("Target Right Speed (m/s)", rightSpeedMetersPerSec);
    double leftSpeedPercent = 0;
    double rightSpeedPecent = 0;
    leftSpeedPercent = m_feedforward.calculate(leftSpeedMetersPerSec) + m_speedController.calculate(getLeftSpeed(), rightSpeedMetersPerSec);
    if(leftSpeedMetersPerSec == 0) {
      leftSpeedPercent = 0;
    }

    rightSpeedPecent = m_feedforward.calculate(rightSpeedMetersPerSec) + m_speedController.calculate(getRightSpeed(), rightSpeedMetersPerSec);
    if(rightSpeedMetersPerSec == 0) {
      rightSpeedPecent = 0;
    }
    m_leftMotor.set(leftSpeedPercent);
    m_rightMotor.set(rightSpeedPecent);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_prevLeftDist = 0;
    m_prevRightDist = 0; 
    m_LeftSpeed = 0;
    m_RightSpeed = 0;
    m_LeftSpeedAvg.reset();
    m_RightSpeedAvg.reset(); 
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return ((m_gyro.getAngleZ() - mTimer.get() * kGyroDrift) % 360 + 360) % 360;
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
    mTimer.restart();
  }

  public double getLeftSpeed() {
    return m_LeftSpeed;
  }

  public double getRightSpeed() {
    return m_RightSpeed;
  }

  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return new ChassisSpeeds(
      getLeftSpeed(),
      getRightSpeed(),
      Rotation2d.fromDegrees(m_gyro.getRateZ()).getRadians());
  }

  public void resetPose(Pose2d pPose) {
    SmartDashboard.putNumber("Last X", pPose.getX());
    SmartDashboard.putNumber("Last Y", pPose.getY());
    SmartDashboard.putNumber("Last Angle", pPose.getRotation().getDegrees());
    System.out.println("Pose Reset");
    mOdometry.resetPosition(
      Rotation2d.fromDegrees(getGyroAngleZ()),
      new DifferentialDriveWheelPositions(
        getLeftDistanceMeters(),
        getRightDistanceMeters()),
      pPose);
  }


  @Override
  public void periodic() {
    mOdometry.update(Rotation2d.fromDegrees(getGyroAngleZ()), new DifferentialDriveWheelPositions(getLeftDistanceMeters(), getRightDistanceMeters()));
    double leftDist = m_leftEncoder.getDistance();
    double rightDist = m_rightEncoder.getDistance();
    m_LeftSpeed = m_LeftSpeedAvg.calculate((leftDist - m_prevLeftDist) * kPeriodicHz);
    m_RightSpeed = m_RightSpeedAvg.calculate((rightDist - m_prevRightDist) * kPeriodicHz);
    m_prevLeftDist = leftDist;
    m_prevRightDist = rightDist;

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left distance (m)", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("Right distance (m)", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Left speed (m/s)", m_LeftSpeed);
    SmartDashboard.putNumber("Right speed (m/s)", m_RightSpeed);
    SmartDashboard.putNumber("Odom X (m)", mOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odom Y (m)", mOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Odom Heading (Deg)", mOdometry.getPoseMeters().getRotation().getDegrees());
  }
}
