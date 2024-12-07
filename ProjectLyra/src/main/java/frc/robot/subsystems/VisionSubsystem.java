package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import java.util.function.Consumer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static frc.robot.Constants.ShooterConstants.kBlueSpeaker;
import static frc.robot.Constants.ShooterConstants.kRedSpeaker;
import static frc.robot.Constants.VisionConstants.*;

// This subsystem controls the vision sensor for the robot.
// This robot has two PhotonVision cameras, one on the front and one on the back.
public class VisionSubsystem extends SubsystemBase
{
    // TODO: Maybe put these in constructor with a check for `Robot.isReal()`
    // Declare variables. Outputs an estimated pose to the consumer.
    // private final PhotonCamera mFrontCamera = new PhotonCamera(kFrontCam);   
    private final PhotonCamera mBackCamera = new PhotonCamera(kBackCam);
    private final Field2d mField = new Field2d();
    private final AprilTagFieldLayout mFieldLayout;
    private final Consumer<Pose2d> mConsumer;
    private final Optional<Alliance> mWorkingAlliance = DriverStation.getAlliance();

    // Initialize the vision subsystem.
    public VisionSubsystem(AprilTagFieldLayout pFieldLayout, Consumer<Pose2d> pConsumer)
    {
        mFieldLayout = pFieldLayout;
        mConsumer = pConsumer;
        SmartDashboard.putData("April Tag Pose", mField);
    }

    // Update the vision estimate with a given consumer as an output.
    // Called continuously via `periodic()` with the default consumer but can also be called
    // additionally by commands if they require vision.
    public void processVisionUpdate(Consumer<Pose2d> pConsumer)
    {
        // PhotonPipelineResult frontCamResult = mFrontCamera.getLatestResult();
        // PhotonPipelineResult backCamResult = mBackCamera.getLatestResult();
        // boolean hasTargets = frontCamResult.hasTargets() || backCamResult.hasTargets();

        // if (!hasTargets) return;

        // final PhotonTrackedTarget bestTarget;
        // final Transform3d cameraLocation;

        // if (frontCamResult.hasTargets())
        // {
        //     bestTarget = frontCamResult.getBestTarget();
        //     cameraLocation = kFrontCamLoc;
        // }
        // else
        // {
        //     bestTarget = backCamResult.getBestTarget();
        //     cameraLocation = kBackCamLoc;
        // }

        // Optional<Pose3d> tagPose = mFieldLayout.getTagPose(bestTarget.getFiducialId());
        // tagPose.ifPresent(pPose -> {
        //     Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(bestTarget.getBestCameraToTarget(), pPose, cameraLocation);
        //     Pose2d robotPose2d = new Pose2d(robotPose3d.getTranslation().toTranslation2d(), robotPose3d.getRotation().toRotation2d());
        //     pConsumer.accept(robotPose2d);
        //     mField.setRobotPose(robotPose2d);
        // });
    }

    // public PhotonPipelineResult getFrontCameraResult() {
    //     return mFrontCamera.getLatestResult();
    // }
 
    public PhotonPipelineResult getBackCameraResult() {
        return mBackCamera.getLatestResult();
    }

    public List<PhotonTrackedTarget> getBackTargets() {
        return getBackCameraResult().getTargets();
    }

    public int getSpeakerToDetect() {
        if (DriverStation.getAlliance().isPresent())
        {
            if (DriverStation.getAlliance().get() == Alliance.Red)
            {
                return kRedSpeaker;
            } else if (DriverStation.getAlliance().get() == Alliance.Blue)
            {
                return kBlueSpeaker;
            }
        }
        return kRedSpeaker;
    }

    public double getSpeakerTargetPitch() {
        for (PhotonTrackedTarget tar : getBackTargets()) {
            if (tar.getFiducialId() == getSpeakerToDetect()) {
                return tar.getPitch();
            }
        }
        return 0.0;
    }

    @Override
    public void periodic()
    {
        processVisionUpdate(mConsumer);
    }
}
