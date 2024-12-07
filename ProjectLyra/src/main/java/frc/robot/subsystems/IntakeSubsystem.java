package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.EArmState;
import frc.robot.Constants.IntakeConstants.EIntakeState;
import frc.robot.Constants.ShooterConstants.EAcceleratorState;
import frc.robot.Robot;
import frc.robot.SystemState;

import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.CommonConstants.*;

import java.util.function.Supplier;

/**
 *  This subsystem controls the intake.
 */ 
public class IntakeSubsystem extends SubsystemBase
{
    // The physical motor. Will be set to `null` when in simulation.
    private final TalonFX mIntakeMotor;
    
    // The beam detector for the intake. Detects if a note is currently inside the intake.
    private final DigitalInput mIntakeNoteDetector;

    private final SystemState mSystemState;

    private final Supplier<Double> mTestIntakeSupplier;
    private final boolean mTest;

    // The intake state
    private EIntakeState mIntakeState = EIntakeState.kStop;

    private boolean mIsTransferringNote = false;

    private boolean mEnabled = true;

    private double mIntakePosBeforeNote = 0;

    /**
     * Creates the intake subsystem.
     * 
     * @param pSystemState The SystemState variable 
     */
    public IntakeSubsystem(SystemState pSystemState, Supplier<Double> pTestIntakeSupplier, boolean pTest)
    {
        mSystemState = pSystemState;
        mTestIntakeSupplier = pTestIntakeSupplier;
        mTest = pTest;
        mSystemState.setIntakeSupplier(() -> mIntakeState);
    
        // Enable control of the related motors. In simulation, they don't exist, so they're set to `null`.
        if (Robot.isReal())
        {
            mIntakeMotor = new TalonFX(kIntakeId);
            mIntakeMotor.getConfigurator().apply(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(20)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(20)
                    .withSupplyCurrentThreshold(0)
                    .withSupplyTimeThreshold(0)
                    .withSupplyCurrentLimitEnable(true));
            mIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
            mIntakeNoteDetector = new DigitalInput(kIntakeNoteDetectorPort);
        }
        else
        {
            mIntakeMotor = null;
            mIntakeNoteDetector = null;
        }
    }

    public IntakeSubsystem(SystemState pSystemState) {
        this(pSystemState, ()-> 0.0, false);
    }

    // Returns `true` if the IR sensor detects a note in the intake.
    public boolean hasNoteInside()
    {
        if (mIntakeNoteDetector == null)
        {
            // Unknown, default to "no".
            return false;
        }
        else
        {
            boolean hasNote = !mIntakeNoteDetector.get();
            if(!hasNote) {
                mIntakePosBeforeNote = mIntakeMotor.getPosition().getValueAsDouble();
            }
            // If the beam is complete, there is no note impeding it.
            return !mIntakeNoteDetector.get();
        }
    }

    public void setIntakeVoltage(double pVoltage) {
        mIntakeMotor.setVoltage(pVoltage);
    }

    public void setEnabled(boolean pEnabled) {
        mEnabled = pEnabled;
    }

    public boolean isEnabled() {
        return mEnabled;
    }

    // Called over and over again.
    @Override
    public void periodic()
    {
        if(mTest) {
            if(!hasNoteInside()) {
                mIntakeMotor.setVoltage(mTestIntakeSupplier.get() * kMaxVolts);
            } else {
                mIntakeMotor.setVoltage(0);
            }
        } else {

            switch(mSystemState.getRobotState()) {
                case kIntakeAndHold:
                    mIntakeState = processIntakeAndHold();
                    break;
                case kIntakeToAccelerator:
                    mIntakeState = processIntakeToAccelerator();
                    break;
                case kOuttakeFloor:
                    mIntakeState = processOutakeFloor();
                    break;
                case kOuttake:
                    mIntakeState = EIntakeState.kOuttake;
                    break;
                case kDefault:
                case kHold:
                    mIntakeState = EIntakeState.kStop;
                    mIsTransferringNote = false;
                    break;
                default:
                    break;
            }
            // SmartDashboard.putString("Intake State: ", mIntakeState.name());

            if (Robot.isReal() && mIntakeMotor != null) // Null check is sanity check.
            {
                // Evaluate and apply our next voltage based on our current intake state
                double currentVoltage = 0;
                switch (mIntakeState) {
                    case kIntake:
                        currentVoltage = -kIntakeVoltage;
                        break;
                    case kFeedAccelerator:
                        currentVoltage = kIntakeVoltage;
                        break;
                    case kOuttake:
                        currentVoltage = -kOutakeVoltage;
                        break;
                    case kHoldingNote:
                        if(Math.abs(mIntakePosBeforeNote - mIntakeMotor.getPosition().getValueAsDouble()) < kAdditionalDistance) {
                            currentVoltage = -kIntakeVoltage;
                        } else {
                            currentVoltage = 0;
                        }
                        break;
                    case kStop:
                        currentVoltage = 0;
                        break;
                    default:
                        break;
                }

                if(mEnabled) {
                    mIntakeMotor.setVoltage(currentVoltage);
                }
                // Update the dashboard with our current voltage.
                SmartDashboard.putNumber("Intake Voltage", currentVoltage);
            }
        }
        SmartDashboard.putBoolean("Intake Has Note", hasNoteInside());
    }

    private EIntakeState processIntakeToAccelerator() {

        if(mSystemState.getAcceleratorState() == EAcceleratorState.kHoldingNote) {
            mIsTransferringNote = false;
        }
    
        if(!hasNoteInside() && mSystemState.getAcceleratorState() != EAcceleratorState.kHoldingNote
            && !mIsTransferringNote) {
            return EIntakeState.kIntake;
        }
    
        if(hasNoteInside() && mSystemState.getArmState() != EArmState.kAcceleratorTransfer) {
            return EIntakeState.kHoldingNote;
        }
    
        if((mSystemState.getArmState() == EArmState.kAcceleratorTransfer 
            && hasNoteInside()) ||
            mIsTransferringNote) {
            mIsTransferringNote = true;
            return EIntakeState.kFeedAccelerator;
        }

        return EIntakeState.kStop;
    }

    private EIntakeState processIntakeAndHold() {
        if(!hasNoteInside()) {
            return EIntakeState.kIntake;
        }

        if(hasNoteInside()) {
            return EIntakeState.kHoldingNote;
        }

        return EIntakeState.kStop;
    }

    private EIntakeState processOutakeFloor() {
        if(mSystemState.getArmState() == EArmState.kIntake && hasNoteInside()) {
            return EIntakeState.kFeedAccelerator;
        }

        return EIntakeState.kStop;
    }
}
