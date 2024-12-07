package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.EArmState;
import frc.robot.Constants.IntakeConstants.EIntakeState;
import frc.robot.Constants.ShooterConstants.EAcceleratorState;
import frc.robot.Robot;
import frc.robot.RobotUtil;
import frc.robot.SystemState;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.CommonConstants.*;
import static frc.robot.Constants.ShooterConstants.kPivotOffset;
import static frc.robot.Constants.ShooterConstants.kPivotRotation;

import java.util.function.Supplier;

// This subsystem controls the arm for the shooter.
// We're going with a similar approach to last year, where we have a shoulder and
// a wrist which can both be independently controlled. The shooter will be atop the wrist
// (the "hand").
public class ArmSubsystem extends SubsystemBase
{    
    // The shoulder motor, encoder, and PID controller.
    // Will be set to `null` during simulation. 
    private final CANSparkMax mShoulderLeft;
    private final CANSparkMax mShoulderRight;
    private final DutyCycleEncoder mLeftShoulderEncoder;
    private final DutyCycleEncoder mRightShoulderEncoder;
    private final ProfiledPIDController mLeftShoulderController;
    private final ProfiledPIDController mRightShoulderController;

    // The wrist motor, encoder, and PID controller.
    // Will also be set to `null` during simulation.
    private final CANSparkMax mWristRight;
    private final CANSparkMax mWristLeft;
    private final ProfiledPIDController mRightWristController;
    private final ProfiledPIDController mLeftWristController;
    private final SystemState mSystemState;
    private final Supplier<Double> mTestShoulderSupplier;
    private final Supplier<Double> mTestWristSupplier;
    private final boolean mTestMode;

    private EArmState mArmState = EArmState.kAcceleratorTransfer;

    // The target wrist and shoulder positions in degrees.
    private double mWristAngleDegrees = kDefaultWristDegrees;
    private double mShoulderAngleDegrees = kDefaultShoulderDegrees;
    private boolean mShoulderReachedAmpTarget = false;
    private boolean mWristReachedDefaultTarget = false;
    private boolean mArmStatesEnabled = true;

    public ArmSubsystem(SystemState pSystemState) {
        this(pSystemState, () -> 0.0, () -> 0.0, false);
    }


    // Initialize the arm subsystem.
    public ArmSubsystem(
        SystemState pSystemState,
        Supplier<Double> pTestShoulderSupplier,
        Supplier<Double> pTestWristSupplier,
        boolean pTestMode)
    {

        mSystemState = pSystemState;
        mSystemState.setArmSupplier(() -> mArmState);
        mTestShoulderSupplier = pTestShoulderSupplier;
        mTestWristSupplier = pTestWristSupplier;
        mTestMode = pTestMode;
    
        if (Robot.isReal())
        {
            // Assign motors.
            mWristLeft = new CANSparkMax(kWristLeftId, MotorType.kBrushless);
            mWristRight = new CANSparkMax(kWristRightId, MotorType.kBrushless);
            mShoulderLeft = new CANSparkMax(kShoulderLeftId, MotorType.kBrushless);
            mShoulderRight = new CANSparkMax(kShoulderRightId, MotorType.kBrushless);

            mShoulderLeft.enableVoltageCompensation(12);
            mShoulderRight.enableVoltageCompensation(12);
            mShoulderLeft.setSmartCurrentLimit(30);
            mShoulderRight.setSmartCurrentLimit(30);
            mShoulderLeft.setOpenLoopRampRate(kShoulderRamp);
            mShoulderRight.setOpenLoopRampRate(kShoulderRamp);
            mShoulderLeft.setIdleMode(IdleMode.kBrake);
            mShoulderRight.setIdleMode(IdleMode.kBrake);
            mLeftShoulderEncoder = new DutyCycleEncoder(kLeftShoulderEncoderId);
            mRightShoulderEncoder = new DutyCycleEncoder(kRightShoulderEncoderId);

            mWristLeft.enableVoltageCompensation(12);
            mWristRight.enableVoltageCompensation(12);
            mWristLeft.setSmartCurrentLimit(30);
            mWristRight.setSmartCurrentLimit(30);
            mWristLeft.setOpenLoopRampRate(kWristRamp);
            mWristRight.setOpenLoopRampRate(kWristRamp);
            mWristLeft.setIdleMode(IdleMode.kBrake);
            mWristRight.setIdleMode(IdleMode.kBrake);

            mWristRight.getAbsoluteEncoder().setPositionConversionFactor(kRotation);
            mWristRight.getAbsoluteEncoder().setInverted(true);
            mWristRight.getAbsoluteEncoder().setZeroOffset(kRightWristOffset);
            mWristLeft.getAbsoluteEncoder().setPositionConversionFactor(kRotation);
            mWristLeft.getAbsoluteEncoder().setZeroOffset(kLeftWristOffset);

            // mLeftShoulderEncoder.setDistancePerRotation(kRotation);
            // mLeftShoulderEncoder.setPositionOffset(kLeftShoulderOffset);
            // mRightShoulderEncoder.setDistancePerRotation(kRotation);
            // mRightShoulderEncoder.setPositionOffset(kRightShoulderOffset);
        
            // Assign controllers and set properties.
            mLeftShoulderController = new ProfiledPIDController(
                kShoulderP,
                kShoulderI,
                kShoulderD,
                new TrapezoidProfile.Constraints(
                    kMaxShoulderSpeedDegSec, 
                    kMaxShoulderAccelDegSec));
            mRightShoulderController = new ProfiledPIDController(
                kShoulderP, 
                kShoulderI,
                kShoulderD,
                new TrapezoidProfile.Constraints(
                    kMaxShoulderSpeedDegSec, 
                    kMaxShoulderAccelDegSec));

            mRightWristController = new ProfiledPIDController(
                kWristP,
                kWristI,
                kWristD,
                new TrapezoidProfile.Constraints(
                    kMaxWristSpeedDegSec,
                    kMaxWristAccelDegSec));
            mLeftWristController = new ProfiledPIDController(
                kWristP,
                kWristI,
                kWristD,
                new TrapezoidProfile.Constraints(
                    kMaxWristSpeedDegSec,
                    kMaxWristAccelDegSec));

        }
        else
        {
            // In simulation, there are no real motors, so we don't create them.
            mShoulderLeft = null;
            mShoulderRight = null;
            mLeftShoulderEncoder = null;
            mRightShoulderEncoder = null;
            mLeftShoulderController = null;
            mRightShoulderController = null;
            mWristRight = null;
            mWristLeft = null;
            mRightWristController = null;
            mLeftWristController = null;
        }
    }

    public void reset() {
        mRightWristController.reset(getRightWristAngleDegrees());
        mLeftWristController.reset(getLeftWristAngleDegrees());
        mRightShoulderController.reset(getRightShoulderAngleDegrees());
        mLeftShoulderController.reset(getLeftShoulderAngleDegrees());
    }

    // Update the desired wrist position to `pAngleDegrees`.
    public void setWristAngleDegrees(double pAngleDegrees)
    {
        mWristAngleDegrees = pAngleDegrees;
    }

    // Update the desired shoulder position to `pAngleDegrees`.
    public void setShoulderAngleDegrees(double pAngleDegrees)
    {
        mShoulderAngleDegrees = pAngleDegrees;
    }

    public double getAbsoluteEncoderPosition(SparkAbsoluteEncoder pEncoder) {
        double degrees =((pEncoder.getPosition() % 360) + 360) % 360;
        if (degrees > kWristNormalizedDegrees) {
            return degrees - kRotation;
        } else {
            return degrees;
        }
    }

    // Get the current wrist position in degrees.
    public double getRightWristAngleDegrees()
    {
        if (Robot.isReal())
        {
            return getAbsoluteEncoderPosition(mWristRight.getAbsoluteEncoder());
        }
        else
        {
            return mWristAngleDegrees;
        }
    }

     public double getLeftWristAngleDegrees()
    {
        if (Robot.isReal())
        {
            return getAbsoluteEncoderPosition(mWristLeft.getAbsoluteEncoder());
        }
        else
        {
            return mWristAngleDegrees;
        }
    }

    // Get the current shoulder position in degrees.
    public double getLeftShoulderAngleDegrees()
    {
        if (Robot.isReal())
        {
            double rawShoulderDegrees = getRawShoulderDegrees(mLeftShoulderEncoder);
            return getShoulderDegrees(rawShoulderDegrees, kLeftShoulderOffset);
        }
        else
        {
            return mShoulderAngleDegrees;
        }
    }

    public double getRightShoulderAngleDegrees()
    {
        if (Robot.isReal())
        {
            double rawShoulderDegrees = -getRawShoulderDegrees(mRightShoulderEncoder);
            return getShoulderDegrees(rawShoulderDegrees, kRightShoulderOffset);
        }
        else
        {
            return mShoulderAngleDegrees;
        }
    }


    public void setRightShoulderSpeedPercent(double pPercent) {
        double percent = calculateGravityCorrectedPercent(
            pPercent,
            getRightShoulderAngleDegrees(),
            getRightWristAngleDegrees());
        mShoulderRight.set(percent);
    }

    public void setLeftShoulderSpeedPercent(double pPercent) {
        double percent = -calculateGravityCorrectedPercent(
            pPercent,
            getLeftShoulderAngleDegrees(),
            getLeftWristAngleDegrees());
        mShoulderLeft.set(percent);
    }

    public void setRawShoulderPercent(double pPercent) {
        SmartDashboard.putNumber("Raw Shoulder", pPercent);
        mShoulderLeft.set(-pPercent);
        mShoulderRight.set(pPercent);
    }

    public void setRawWristPercent(double pPercent) {
        SmartDashboard.putNumber("Raw Wrist", pPercent);
        mWristLeft.set(-pPercent);
        mWristRight.set(pPercent);
    }

    public static double calculateGravityCorrectedPercent(double pPercent, double pShoulderAngle, double pWristAngle){
        return pPercent
            + kShoulderG * Math.cos(Math.toRadians(pShoulderAngle))
            + kShoulderExtG * Math.sin(Math.toRadians((pWristAngle + kDefaultShoulderDegrees)/2));
    }

    public void setRightWristSpeedPercent(double pPercent) {
        double combinedAngle = getRightWristAngleDegrees() + (kDefaultShoulderDegrees - getRightShoulderAngleDegrees());
        double percent = pPercent + kWristG * Math.cos(Math.toRadians(combinedAngle));
        mWristRight.set(percent);
    }

    public void setLeftWristSpeedPercent(double pPercent) {
        double combinedAngle = getLeftWristAngleDegrees() + (kDefaultShoulderDegrees - getLeftShoulderAngleDegrees());
        double percent = -(pPercent + kWristG * Math.cos(Math.toRadians(combinedAngle)));
        mWristLeft.set(percent);
    }

    public void setArmStatesEnabled(boolean pEnabled) {
        mArmStatesEnabled = pEnabled;
        if(!pEnabled) {
            mWristLeft.set(0.0);
            mWristRight.set(0.0);
            mShoulderLeft.set(0.0);
            mShoulderRight.set(0.0);
        }
    }

    public boolean isArmStatesEnabled() {
        return mArmStatesEnabled;
    }

    public void updateShoulderAngle() {
        double rightSpeed = mRightShoulderController.calculate(getRightShoulderAngleDegrees(), mShoulderAngleDegrees);
        double leftSpeed = mLeftShoulderController.calculate(getLeftShoulderAngleDegrees(), mShoulderAngleDegrees);
        if(mArmStatesEnabled) {
            setRightShoulderSpeedPercent(rightSpeed);
            setLeftShoulderSpeedPercent(leftSpeed);
        }
    }

    public void updateWristAngle() {
        double rightSpeed = mRightWristController.calculate(getRightWristAngleDegrees(), mWristAngleDegrees);
        double leftSpeed = mLeftWristController.calculate(getLeftWristAngleDegrees(), mWristAngleDegrees);
        if(mArmStatesEnabled) {
            setRightWristSpeedPercent(rightSpeed);
            setLeftWristSpeedPercent(leftSpeed);
        }
    }

    public static double getRawShoulderDegrees(DutyCycleEncoder pEncoder) {
        return pEncoder.getAbsolutePosition() * kRotation;
    }

    public static double getShoulderDegrees(double pRawShoulderDegrees, double pOffset)
    {
        double rawAngle = pRawShoulderDegrees - pOffset;
        double posAngle = ((rawAngle % kRotation) + kRotation) % kRotation;
        if(posAngle > kShoulderNormalizedDegrees) {
            posAngle -= kRotation;
        }
        return posAngle;
    }

    // Called over and over again.
    @Override
    public void periodic()
    {
        if (Robot.isReal())
        {
            if(mTestMode)
            {
                setLeftWristSpeedPercent(0);
                setRightWristSpeedPercent(0);
                setLeftShoulderSpeedPercent(0);
                setRightShoulderSpeedPercent(0);
            }
            else
            {
                mWristAngleDegrees = processWristAngle();
                mShoulderAngleDegrees = processShoulderAngle();
                mArmState = processArmState();
                updateWristAngle();
                updateShoulderAngle();
            }
        }

        SmartDashboard.putNumber("Target Wrist Angle:", mWristAngleDegrees);
        SmartDashboard.putNumber("Target Shoulder Angle:", mShoulderAngleDegrees);
        SmartDashboard.putString("Arm State", mArmState.name());
        SmartDashboard.putNumber("Left Wrist Degrees", getLeftWristAngleDegrees());
        SmartDashboard.putNumber("Right Wrist Degrees", getRightWristAngleDegrees());
        SmartDashboard.putNumber("Left Shoulder Degrees", getLeftShoulderAngleDegrees());
        SmartDashboard.putNumber("Right Shoulder Degrees", getRightShoulderAngleDegrees());
    }

    private boolean wristAtTarget(double pTarget) {
        return RobotUtil.distanceFromTarget(getLeftWristAngleDegrees(), pTarget) <= kDegreesTolerance
            && RobotUtil.distanceFromTarget(getRightWristAngleDegrees(), pTarget) <= kDegreesTolerance;
    }

    private boolean shoulderAtTarget(double pTarget) {
        return RobotUtil.distanceFromTarget(getLeftShoulderAngleDegrees(), pTarget) <= kDegreesTolerance
            && RobotUtil.distanceFromTarget(getRightShoulderAngleDegrees(), pTarget) <= kDegreesTolerance;
    }


    private double processWristAngle()
    {
        switch(mSystemState.getRobotState()) {
            case kIntakeAndHold:
            case kIntakeToAccelerator:
                return processWristIntake();
            case kPositionAmp:
                if(shoulderAtTarget(kAmpShoulderDegrees)) {
                    mShoulderReachedAmpTarget = true;
                }
                if(mShoulderReachedAmpTarget) {
                    return kAmpWristDegrees;
                } else {
                    return kDefaultWristDegrees;
                }
            case kOuttake:
            case kHold:
                return mWristAngleDegrees;
            default:
                break;
        }
        mShoulderReachedAmpTarget = false;
        return kDefaultWristDegrees;
    }

    private double processShoulderAngle() {
        switch(mSystemState.getRobotState()) {
            case kPositionAmp:
                mWristReachedDefaultTarget = false;
                return kAmpShoulderDegrees;
            case kOuttake:
            case kHold:
                return mShoulderAngleDegrees;
            case kDefault:
                if(wristAtTarget(kAmpWristDegrees)) {
                    mWristReachedDefaultTarget = true;
                }
                if(mWristReachedDefaultTarget) {
                    return kDefaultShoulderDegrees;
                } else {
                    return mShoulderAngleDegrees;
                }
            default:
                break;
        }
        return kDefaultShoulderDegrees;
    }

    private double processWristIntake() {
        if(mSystemState.getIntakeState() == EIntakeState.kHoldingNote ||
            mSystemState.getIntakeState() == EIntakeState.kFeedAccelerator ||
            mSystemState.getIntakeState() == EIntakeState.kStop) {
            return kDefaultWristDegrees;
        }
        return kIntakeWristDegrees;
    }

    private EArmState processArmState()
    {
        if(wristAtTarget(kIntakeWristDegrees) && shoulderAtTarget(kDefaultShoulderDegrees))
        {
            return EArmState.kIntake;
        }

        if(wristAtTarget(kDefaultWristDegrees) && shoulderAtTarget(kDefaultShoulderDegrees))
        {
            return EArmState.kAcceleratorTransfer;
        }

        if(wristAtTarget(kAmpWristDegrees) && shoulderAtTarget(kAmpShoulderDegrees)) {
            return EArmState.kAmp;
        }
        return EArmState.kTransition;
    }

    public double getTargetWristAngleDegrees() {
        return mWristAngleDegrees;
    }

    public double getTargetShouldAngleDegrees() {
        return mShoulderAngleDegrees;
    }
}