package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static frc.robot.Constants.SwerveConstants.*;

// The swerve wheel subsystem. 
public class SwerveSubsystem extends SubsystemBase
{
    private final SwerveDrive mSwerveDrive;

    // Read preferences from the swerve config file located at `deploy/swerve/swervedrive.json`
    private static SwerveDrive readSwerveConfig()
    {
        SwerveDrive swerveDrive = null;
        File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
        try
        {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
                new SimpleMotorFeedforward(kS, kV, kA),
                kMaxSpeedMS);
        }
        catch (IOException e)
        {
            throw new RuntimeException(e);
        }
        
        return swerveDrive;
    }

    // Initialized the subsystem.
    public SwerveSubsystem()
    {
        //SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        mSwerveDrive = readSwerveConfig();
        mSwerveDrive.setHeadingCorrection(false);
    
        // Configure the autonomous generator. Reads PathPlanner files.
        AutoBuilder.configureHolonomic(
            mSwerveDrive::getPose,                        // Robot pose supplier
            mSwerveDrive::resetOdometry,                  // Method to reset odometry (will be called if your auto has a starting pose)
            mSwerveDrive::getRobotVelocity,               // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            mSwerveDrive::drive,                          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig(              // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                kMaxSpeedMS,                       // Max module speed, in m/s
                0.4,                      // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig()                    // Default path replanning config. See the API for the options here
            ),
            () ->
            {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                // `true` is red, `false` is blue.
                return isRedAlliance();
            },
            this // Reference to this subsystem to set requirements
        );
    }

    private static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent())
        {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    // Drive in some direction with its reference point set to the field itself.
    public void driveFieldOriented(ChassisSpeeds pVelocity)
    {
        if(isRedAlliance())
        {
            ChassisSpeeds fieldOrientedVelocity =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    pVelocity,
                    mSwerveDrive.getYaw().plus(Rotation2d.fromRadians(Math.PI)));
            mSwerveDrive.drive(fieldOrientedVelocity);
        }
        else
        {
            mSwerveDrive.driveFieldOriented(pVelocity);
        }
    }

    // Drive in some direction with its reference point set to the robot.
    public void driveRobotOriented(ChassisSpeeds pVelocity)
    {
        mSwerveDrive.drive(pVelocity);
    }

    // Reset the odometry.
    public void resetOdometry(Pose2d pPose)
    {
        mSwerveDrive.setGyro(new Rotation3d(0, 0, pPose.getRotation().getRadians()));
        mSwerveDrive.resetOdometry(pPose);
    }

    // Reset the gyroscope.
    public void resetHeading()
    {
        Rotation2d newHeading = Rotation2d.fromRadians(0);
        if(isRedAlliance())
        {
            newHeading = Rotation2d.fromRadians(Math.PI);
        }
        resetOdometry(new Pose2d(getPose().getTranslation(), newHeading));
    }

    // Get the current heading in degrees.
    public double getHeadingDegrees()
    {
        return mSwerveDrive.getPose().getRotation().getDegrees();
    }
    
    // Get the estimated position and rotation of the robot.
    public Pose2d getPose()
    {
        return mSwerveDrive.getPose();
    }

    // Update the estimate of the robot's pose based on a vision measurement.
    public void updatePoseFromVision(Pose2d pVisionPose)
    {
        mSwerveDrive.addVisionMeasurement(pVisionPose, Timer.getFPGATimestamp());
    }
}
