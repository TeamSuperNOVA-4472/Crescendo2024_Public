package frc.robot.commands.autos;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.SwerveSubsystem;

// Represents a path to follow created by PathPlanner.
public class PathCommand extends SequentialCommandGroup
{

    private final PathPlannerPath mPath;

    // Prepare the autonomous for use.
    public PathCommand(String pPath, SwerveSubsystem pSwerveSubsystem)
    {
        // Read the path file from PathPlanner and flip the alliance if required.
        mPath = PathPlannerPath.fromPathFile(pPath);

        addCommands // Reset odometry, wait for it to process (takes time), and follow the given path.
        (
            new InstantCommand(()->  pSwerveSubsystem.resetOdometry(getStartingPose())),
            new WaitCommand(0.1),
            AutoBuilder.followPath(mPath)
        );
    }

    private Pose2d getStartingPose() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red)
        {
            return mPath.flipPath().getPreviewStartingHolonomicPose();
        }
        return mPath.getPreviewStartingHolonomicPose();
    }
}
