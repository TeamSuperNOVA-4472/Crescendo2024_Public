package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

// A command that will turn the robot using its swerve subsystem to a desired
// target determined with vision.
public class VisionTurnTo extends Command
{
    // The PID controller.
    private final PIDController mPIDController;

    // Subsystems used.
    private final SwerveSubsystem mSwerveSubsystem;
    private final VisionSubsystem mVisionSubsystem;

    private final Timer mTimer;
    private final double mTimeout;

    private boolean mTargetSeen;
    private double mTargetAngle;
    private boolean mFacingTarget;

    private List<PhotonTrackedTarget> trackedTargets;
    private PhotonTrackedTarget trackedTarget;

    // Prepares the subsystem for use.
    public VisionTurnTo(SwerveSubsystem pSwerveSubsystem, VisionSubsystem pVisionSubsystem, double pTimeout)
    {
        // Create the PID controller. Could make the kP, kI, and kD constants in the `Constants.java` file.
        mPIDController = new PIDController
        (
            .1, 0, 0.005
            // ,
            // new TrapezoidProfile.Constraints
            // (
            //     0.275 * Constants.SwerveConstants.kMetersPerSecondToDegreesPerSecond,
            //     0.275 * Constants.SwerveConstants.kMetersPerSecondToDegreesPerSecond
            // )
        );
        mPIDController.enableContinuousInput(0, 360);

        // Assign subsystems.
        mSwerveSubsystem = pSwerveSubsystem;
        mVisionSubsystem = pVisionSubsystem;

        mTimer = new Timer();
        //mPIDController.enableContinuousInput(-180, 180);
        mTimeout = pTimeout;

        // Declare the swerve and vision subsystems as requirements.
        addRequirements(pSwerveSubsystem, pVisionSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        mTimer.reset();
        mTimer.start();
        mFacingTarget = false;
        mTargetSeen = false;
        mTargetAngle = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        SmartDashboard.putNumber("Angle To Target", mTargetAngle);
        if (mVisionSubsystem.getBackCameraResult().hasTargets() && !mTargetSeen)
        {
            trackedTargets = mVisionSubsystem.getBackTargets();
            for (PhotonTrackedTarget tar : trackedTargets) {
                
                if (tar.getFiducialId() == mVisionSubsystem.getSpeakerToDetect()) {
                    trackedTarget = tar;
                    mTargetSeen = true;
                }
            }
            if(mTargetSeen) {
                mTargetAngle = mSwerveSubsystem.getHeadingDegrees() - trackedTarget.getYaw();
            }
        }

        if(mTargetSeen) {
            double currentAngle = mSwerveSubsystem.getHeadingDegrees();
            double rotationalVelocity = mPIDController.calculate(mSwerveSubsystem.getHeadingDegrees(),
                mTargetAngle + Constants.VisionConstants.kAprilTagOffsetDegs);
            double error = Math.abs(mTargetAngle - currentAngle);
            if (error < 1.0)
            {
                rotationalVelocity = 0;
                mFacingTarget = true;
            }
            SmartDashboard.putNumber("Auto Rot Error", error);
            SmartDashboard.putNumber("Auto Rot", rotationalVelocity);

            // Include default drive logic to enable strafing while aiming
            mSwerveSubsystem.driveRobotOriented(new ChassisSpeeds(0.0, 0.0, rotationalVelocity));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {
        mTimer.stop();
        mSwerveSubsystem.driveRobotOriented(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return mTimer.get() >= mTimeout || mFacingTarget;
    }
}
