package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimDrive extends SubsystemBase {
    private static final double CYCLE_HZ = 50;
    private static final double MAX_TURN_RATE_DEG = 360;
    private static final double MAX_TURN_RATE_DEG_PER_CYCLE = MAX_TURN_RATE_DEG / CYCLE_HZ;
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double MAX_SPEED_M_S_PER_CYCLE = MAX_SPEED_M_S / CYCLE_HZ;
    private final Field2d mField = new Field2d();

    private final DifferentialDriveOdometry mDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

    private double mLeftPos = 0;
    private double mRightPos = 0;
    private Rotation2d mAngle = Rotation2d.fromDegrees(0);
    private Pose2d mPose = new Pose2d();

    public SimDrive() {
        SmartDashboard.putData("Field", mField);
    }

    public void setPose(Pose2d pPose) {
        mField.setRobotPose(pPose);
        mLeftPos = 0;
        mRightPos = 0;
        mAngle = pPose.getRotation();
        mDriveOdometry.resetPosition(mAngle, mLeftPos, mRightPos, pPose);
    }

    public void driveFieldOrientedSwerve(double pSpeedX, double pSpeedY, double pTurnZ) {
        double newPosX = mPose.getX() + MAX_SPEED_M_S_PER_CYCLE * pSpeedX;
        double newPosY = mPose.getY() + MAX_SPEED_M_S_PER_CYCLE * pSpeedY;
        mAngle = mAngle.plus(Rotation2d.fromDegrees(MAX_TURN_RATE_DEG_PER_CYCLE * pTurnZ));
        mPose = new Pose2d(newPosX, newPosY, mAngle);
    }

    public void driveArcade(double pSpeedX, double pTurnZ) {
        WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(pSpeedX, pTurnZ, false);
        mLeftPos += MAX_SPEED_M_S_PER_CYCLE * wheelSpeeds.left;
        mRightPos += MAX_SPEED_M_S_PER_CYCLE * wheelSpeeds.right;
        mAngle = mAngle.plus(Rotation2d.fromDegrees(MAX_TURN_RATE_DEG_PER_CYCLE * pTurnZ));
        mDriveOdometry.update(mAngle, mLeftPos, mRightPos);
        mPose = mDriveOdometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        mField.setRobotPose(mPose);
    }

}
