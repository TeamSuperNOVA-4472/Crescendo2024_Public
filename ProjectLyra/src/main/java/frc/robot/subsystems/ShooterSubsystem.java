package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.EArmState;
import frc.robot.Constants.IntakeConstants.EIntakeState;
import frc.robot.Constants.ESystemState;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ShooterConstants.EAcceleratorState;
import frc.robot.Constants.ShooterConstants.EShooterPivotState;
import frc.robot.Robot;
import frc.robot.RobotUtil;
import frc.robot.SystemState;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.CommonConstants.*;

import edu.wpi.first.math.MathUtil;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

// The shooter subsystme. Uses a flywheel motor and angle motor.
public class ShooterSubsystem extends SubsystemBase
{
    // Motors, motor encoders, and motor controllers.
    private final TalonFX mFlywheelTop;
    private final TalonFX mFlywheelBot;
    private final CANSparkMax mAccelerator;
    private final CANSparkMax mAngleMotor;
    private final DutyCycleEncoder mAngleEncoder;
    private final PIDController mAngleController;
    private final VelocityVoltage mRequest = new VelocityVoltage(0).withSlot(0);

    // The beam detector for the intake. Detects if a note is currently inside the intake.
    private final DigitalInput mAcceleratorNoteDetector;

    private final SystemState mSystemState;

    private final Supplier<Double> mTestFlywheelSupplier;
    private final Supplier<Double> mTestAcceleratorSupplier;
    private final Supplier<Double> mTestShooterPivotSupplier;
    private final Supplier<Double> mVisionPitchSupplier;
    private final boolean mTestMode;

    private double mAngleDegrees; 
    private double mFlywheelSpeedRPM;

    private EShooterPivotState mShooterPivotState = EShooterPivotState.kCloseShot;
    private EShooterPivotState mShooterMode = EShooterPivotState.kVisionShot;

    private EAcceleratorState mAcceleratorState = EAcceleratorState.kStop;

    // Initialize the shooter.
    public ShooterSubsystem(SystemState pSystemState,
        Supplier<Double> pVisionPitchSupplier,
        Supplier<Double> pTestFlywheelSupplier,
        Supplier<Double> pTestAcceleratorSupplier,
        Supplier<Double> pTestShooterPivotSupplier,
        boolean pTestMode)
    {

        mSystemState = pSystemState;
        mTestFlywheelSupplier = pTestAcceleratorSupplier;
        mTestAcceleratorSupplier = pTestAcceleratorSupplier;
        mTestShooterPivotSupplier = pTestShooterPivotSupplier;
        mVisionPitchSupplier = pVisionPitchSupplier;
        mTestMode = pTestMode;

        mSystemState.setShooterPivotSupplier(() -> mShooterPivotState);
        mSystemState.setAcceleratorSupplier(() -> mAcceleratorState);
        if (Robot.isReal())
        {
            mFlywheelTop = new TalonFX(kFlywheelTopId);
            mFlywheelBot = new TalonFX(kFlywheelBotId);
            mAccelerator = new CANSparkMax(kAcceleratorId, MotorType.kBrushless);
            mAngleMotor = new CANSparkMax(kAngleMotorId, MotorType.kBrushless);
            mAngleEncoder = new DutyCycleEncoder(kAngleEncoderId);
            mAngleController = new PIDController(kAngleP, kAngleI, kAngleD);
            mAcceleratorNoteDetector = new DigitalInput(kAcceleratorNoteDetectorPort);
            
            

            mFlywheelTop.getConfigurator().apply(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(40)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentThreshold(0)
                    .withSupplyTimeThreshold(0)
                    .withSupplyCurrentLimitEnable(true));
            mFlywheelBot.getConfigurator().apply(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(40)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentThreshold(0)
                    .withSupplyTimeThreshold(0)
                    .withSupplyCurrentLimitEnable(true));

            var topSlot0Configs = new Slot0Configs();
            topSlot0Configs.kS = kFlywheelS; // Add 0.05 V output to overcome static friction
            topSlot0Configs.kV = kFlywheelV; // A velocity target of 1 rps results in 0.12 V output
            topSlot0Configs.kP = kFlywheelP; // An error of 1 rps results in 0.11 V output
            topSlot0Configs.kI = 0; // no output for integrated error
            topSlot0Configs.kD = 0; // no output for error derivative

            mFlywheelTop.getConfigurator().apply(topSlot0Configs);

            var botSlot0Configs = new Slot0Configs();
            botSlot0Configs.kS = kFlywheelS; // Add 0.05 V output to overcome static friction
            botSlot0Configs.kV = kFlywheelV; // A velocity target of 1 rps results in 0.12 V output
            botSlot0Configs.kP = kFlywheelP; // An error of 1 rps results in 0.11 V output
            botSlot0Configs.kI = 0; // no output for integrated error
            botSlot0Configs.kD = 0; // no output for error derivative

            mFlywheelBot.getConfigurator().apply(botSlot0Configs);

            mAngleEncoder.getAbsolutePosition();
            mAngleEncoder.setPositionOffset(kPivotOffset/kPivotRotation);
            mAngleMotor.setSmartCurrentLimit(20);
            mAngleMotor.enableVoltageCompensation(12);
            mAngleMotor.setIdleMode(IdleMode.kBrake);
        }
        else
        {
            // If we're using simulation, none of the motors exist.
            mFlywheelTop = null;
            mFlywheelBot = null;
            mAccelerator = null;
            mAngleMotor = null;
            mAngleEncoder = null;
            mAngleController = null;
            mAcceleratorNoteDetector = null;
        }
        mAngleDegrees = 0;
        mFlywheelSpeedRPM = kIdleRpm;
    }

    public ShooterSubsystem(SystemState pSystemState, Supplier<Double> pVisionPitchSupplier)
    {
        this(pSystemState, pVisionPitchSupplier, () -> 0.0, () -> 0.0, () -> 0.0, false);
    }

    // Set the flywheel speed to a specific RPM.
    public void setFlywheelSpeed(double pFlywheelSpeedRPM)
    {
        mFlywheelSpeedRPM = pFlywheelSpeedRPM;       
    }

    // Set the flywheel angle to a specific rotation.
    public void setAngleDegrees(double pAngleDegrees)
    {
        mAngleDegrees = pAngleDegrees;
    }

    public void setShooterMode(EShooterPivotState pPivotState)
    {
        mShooterMode = pPivotState;
    }
    
    // Get the *desired* flywheel RPM.
    public double getTargetFlywheelRPM()
    {
        return mFlywheelSpeedRPM;
    }

    // Get the *desired* flywheel angle.
    public double getTargetAngleDegrees()
    {
        return mAngleDegrees;
    }

    public double getTopFlywheelRpm() {
        return mFlywheelTop.getRotorVelocity().getValueAsDouble() * 60;
    }

    public double getBotFlywheelRpm() {
        return mFlywheelBot.getRotorVelocity().getValueAsDouble() * 60;
    }

    // Get the *current* flywheel angle.
    // Identical to `getTargetFlywheelAngle()` if in simulation.
    public double getActualAngleDegrees()
    {
        if (Robot.isReal())
        {
            double rawAngle = -(mAngleEncoder.getAbsolutePosition() * kPivotRotation - kPivotOffset);
            double posAngle = ((rawAngle % kPivotRotation) + kPivotRotation) % kPivotRotation;
            if(posAngle > 90) {
                posAngle -= kPivotRotation;
            }
            return posAngle;
        }
        else
        {
            return mAngleDegrees;
        }
    }

    // Returns `true` if the IR sensor detects a note in the intake.
    public boolean hasNoteInside()
    {
        if (mAcceleratorNoteDetector == null)
        {
            // Unknown, default to "no".
            return false;
        }
        else
        {
            // If the beam is complete, there is no note impeding it.
            return !mAcceleratorNoteDetector.get();
        }
    }

    public EShooterPivotState getShooterMode() {
        return mShooterMode;
    }

    private void setAngleMotorSpeedPercent(double pPercent) {
        double overallPercent = pPercent + kAngleG * Math.cos(Math.toRadians(pPercent));
        mAngleMotor.set(overallPercent);
    }

    @Override
    public void periodic()
    {

        if(mTestMode) {
            // mAccelerator.set(mTestAcceleratorSupplier.get());
            // setFlywheelSpeedPercent(mTestFlywheelSupplier.get());
            // double output = mAngleController.calculate(getActualAngleDegrees(), 20);
            // setAngleMotorSpeedPercent(output);
        } else {
            switch(mSystemState.getRobotState()) {
                case kIntakeAndHold:
                    mAcceleratorState = processIntakeAndHoldForAccelerator();
                    mShooterPivotState = processIntakeAndHoldForShooterPivot();
                break;
                case kIntakeToAccelerator:
                    mAcceleratorState = processIntakeToAcceleratorForAccelerator();
                    mShooterPivotState = processIntakeToAcceleratorForShooterPivot();
                break;
                case kShoot:
                    mAcceleratorState = EAcceleratorState.kToShooter;
                break;
                case kDefault:
                case kHold:
                    mAcceleratorState = EAcceleratorState.kStop;
                    mShooterPivotState = mShooterMode;
                break;
                default:
                break;
            }

            // Update the flywheel RPM and speed based on what's been desired.
            if (Robot.isReal())
            {

                // Set fly wheel speed
                // mFlywheelBot.set(0);
                // mFlywheelTop.set(0);
                mFlywheelTop.setControl(mRequest.withVelocity(mFlywheelSpeedRPM / kRpsToRpm));
                mFlywheelBot.setControl(mRequest.withVelocity(mFlywheelSpeedRPM / kRpsToRpm));

                // Set accelerator voltage
                double acceleratorVoltage = 0;
                switch(mAcceleratorState) {
                    case kAcceleratorHold:
                        acceleratorVoltage = kAcceleratorHoldVoltage;
                        break;
                    case kToShooter:
                        acceleratorVoltage = kAcceleratorShootVolatage;
                        break;
                    case kToIntake:
                        acceleratorVoltage = -kAcceleratorHoldVoltage;
                        break;
                    case kHoldingNote:
                    case kStop:
                    default:
                        break;
                }
                mAccelerator.setVoltage(acceleratorVoltage);

                mAngleDegrees = getDesiredPivotAngle();
                mShooterPivotState = processPivotState();
                // Set angle motor output
                double angleMotorOutput = mAngleController.calculate(
                    getActualAngleDegrees(),
                    mAngleDegrees);
                setAngleMotorSpeedPercent(angleMotorOutput);
                SmartDashboard.putString("Pivot State", mShooterPivotState.name());
                SmartDashboard.putString("Accelerator State", mAcceleratorState.name());
                SmartDashboard.putNumber("Top Flywheel RPM", getTopFlywheelRpm());
                SmartDashboard.putNumber("Bot Flywheel RPM", getBotFlywheelRpm());
            }
        }

        SmartDashboard.putNumber("Target RPM", getTargetFlywheelRPM());
        SmartDashboard.putNumber("Target Angle", getTargetAngleDegrees());
        SmartDashboard.putNumber("Flywheel Angle", getActualAngleDegrees());
        SmartDashboard.putBoolean("Accelerator has note", hasNoteInside());
    }

    private EShooterPivotState processIntakeToAcceleratorForShooterPivot() {
        if(!hasNoteInside()) {
            return EShooterPivotState.kIntakeTransfer;
        }
        return mShooterMode;
    }

    private EShooterPivotState processIntakeAndHoldForShooterPivot() {
        if(hasNoteInside()) {
            return EShooterPivotState.kIntakeTransfer;
        }
        return mShooterMode;
    }

    private EAcceleratorState processIntakeToAcceleratorForAccelerator() {
        if(mSystemState.getArmState() == EArmState.kAcceleratorTransfer && !hasNoteInside()) {
            return EAcceleratorState.kAcceleratorHold;
        }
    
        if(hasNoteInside()) {
            return EAcceleratorState.kHoldingNote;
        }

        return EAcceleratorState.kStop;
    }

    private EAcceleratorState processIntakeAndHoldForAccelerator() {
        return EAcceleratorState.kStop;
    }

    private double getDesiredPivotAngle()
    {
        if(mSystemState.getRobotState() == ESystemState.kShoot) {
            if (mShooterMode == EShooterPivotState.kVisionShot)
            {
                return getVisionPivotAngle();
            }
            return kPivotCloseDegrees;
        }
        if (!hasNoteInside()) {
            return kPivotIntakeDegrees;
        } else if(mShooterMode == EShooterPivotState.kVisionShot) {
            return getVisionPivotAngle();
        }
        return kPivotCloseDegrees;
    }

    private double getVisionPivotAngle() {
        return MathUtil.clamp(VisionConstants.kPivotMap.get(mVisionPitchSupplier.get()), 0, 55);
    }

    private EShooterPivotState processPivotState()
    {
        double pivotAngle = getActualAngleDegrees();
        if (RobotUtil.distanceFromTarget(pivotAngle,kPivotIntakeDegrees) <= kDegreesTolerance) return EShooterPivotState.kIntakeTransfer;
        else if (RobotUtil.distanceFromTarget(pivotAngle,kPivotCloseDegrees) <= kDegreesTolerance) return EShooterPivotState.kCloseShot;
        else return EShooterPivotState.kTransition;
    }
}
