package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.SwerveConstants.*;

// The swerve teleop command.
public class SwerveTeleop extends Command
{
    // These are suppliers because we don't need to know the whole state of the controller at once.
    private final Supplier<Double> mFwdInput;
    private final Supplier<Double> mSideInput;
    private final Supplier<Double> mTurnInput;
    private final Supplier<Boolean> mResetHeadingInput;
    private final SwerveSubsystem mSwerveSubsystem;

    // The PID that controls the swerve motors.
    private final PIDController mGyroController = new PIDController(0.05, 0, 0.0005);
    private double mTargetHeading;

    // Initialize the command with its subsystem.
    public SwerveTeleop(
        Supplier<Double> pFwdInput,           // Forward Controls
        Supplier<Double> pSideInput,          // Side Controls
        Supplier<Double> pTurnInput,          // Rotation Controls
        Supplier<Boolean> pResetHeadingInput, // Reset Control
        SwerveSubsystem pSwerveSubsystem)
    {
        // Assign values.
        mFwdInput = pFwdInput;
        mSideInput = pSideInput;
        mTurnInput = pTurnInput;
        mResetHeadingInput = pResetHeadingInput;
        mSwerveSubsystem = pSwerveSubsystem;
    
        mTargetHeading = mSwerveSubsystem.getHeadingDegrees();
        mGyroController.enableContinuousInput(0, 360);

        addRequirements(pSwerveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        // Generate speed values.
        double updatedFwdSpeedMS = mFwdInput.get() * kMaxSpeedMS;
        double updatedSideSpeedMS = mSideInput.get() * kMaxSpeedMS;
        double updatedTurnSpeedRadS = mTurnInput.get() * kMetersPerSecondToRadiansPerSecond * kMaxSpeedMS;

        // The rotation speed depends on whether we're moving or not.
        if(updatedTurnSpeedRadS == 0.0 && (updatedFwdSpeedMS != 0 || updatedSideSpeedMS != 0))
        {
            updatedTurnSpeedRadS = mGyroController.calculate(mSwerveSubsystem.getHeadingDegrees(), mTargetHeading);
        }
        else
        {
            mTargetHeading = mSwerveSubsystem.getHeadingDegrees();
        }

        // Generate chassis speed based on individual components.
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(
            updatedFwdSpeedMS,
            updatedSideSpeedMS, 
            updatedTurnSpeedRadS);
        mSwerveSubsystem.driveFieldOriented(updatedSpeeds);

        // If pressing the reset heading button, reset the heading.
        if(mResetHeadingInput.get())
        {
            mSwerveSubsystem.resetHeading();
        }
    }
}
