package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ESystemState;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ShooterConstants.EShooterPivotState;
import frc.robot.Constants.ArmConstants.EArmState;
import frc.robot.Constants.LEDConstants.ELEDState;
import frc.robot.commands.*;
import frc.robot.commands.autos.*;
import frc.robot.subsystems.*;

import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import static frc.robot.Constants.ShooterConstants.*;

// This is where the big stuff happens. We create and use subsystems and commands here.
public class RobotContainer {

    // Declare the controller used. Could also be a `PS4Controller` or `Joystick`
    private final XboxController mDriverController;

    private final XboxController mPartnerController;

    private final SystemState mSystemState;

    // Declare the subsystems and commands that are going to be used.
    private final IntakeSubsystem mIntakeSubsystem;

    private final ClimbSubsystem mClimbSubsystem;
    private final ClimbTeleop mClimbTeleop;

    private final SwerveSubsystem mSwerveSubsystem;
    private final SwerveTeleop mSwerveTeleop;

    private final AprilTagFieldLayout mAprilTagFieldLayout;
    private final ShooterSubsystem mShooterSubsystem;

    private final ArmSubsystem mArmSubsystem;
    private final ArmDefault mArmDefault;
    private final IntakeDefault mIntakeDefault;

    private final LEDs mLEDs;

    private final VisionSubsystem mVisionSubsystem;
    // private final VisionCommand mVisionCommand;

    ShuffleboardTab driverstationTab = Shuffleboard.getTab("Driver Station Tab");
    SendableChooser<Command> autoSelector = new SendableChooser<Command>();
    private final Command mAuto4Note;
    private final Command mAuto2NoteAmpSide;
    private final Command mAuto2NoteCenter;
    private final Command mJustShoot;
    private final Command mAuto4Mid;
    private final Command mAutoLeftMidrush;

    // Triggered on robot start.
    public RobotContainer()
    {
        // Assign the controller.
        mDriverController = new XboxController(OperatorConstants.kDriverControllerPort);
        mPartnerController = new XboxController(OperatorConstants.kPartnerControllerPort);
    
        mSystemState = new SystemState();

        // Assign subsystems.

        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch(IOException e) {
            throw new RuntimeException(e);
        }

        mSwerveSubsystem = new SwerveSubsystem();
        mAprilTagFieldLayout = layout;        
        mVisionSubsystem = new VisionSubsystem(mAprilTagFieldLayout, mSwerveSubsystem::updatePoseFromVision);
        mIntakeSubsystem = new IntakeSubsystem(mSystemState);
        mIntakeDefault = new IntakeDefault(
            mIntakeSubsystem,
            () -> mPartnerController.getRightTriggerAxis() != 0,
            mPartnerController::getRightBumper);
        mIntakeSubsystem.setDefaultCommand(mIntakeDefault);
        mShooterSubsystem = new ShooterSubsystem(mSystemState, () -> mVisionSubsystem.getSpeakerTargetPitch());
        mArmSubsystem = new ArmSubsystem(mSystemState);
        mClimbSubsystem = new ClimbSubsystem();
        mArmDefault = new ArmDefault(
            mArmSubsystem,
            () -> MathUtil.applyDeadband(-mPartnerController.getLeftY(), OperatorConstants.kDeadband),
            () -> MathUtil.applyDeadband(-mPartnerController.getRightY(), OperatorConstants.kDeadband));
        mArmSubsystem.setDefaultCommand(mArmDefault);
        
        mLEDs = new LEDs();

        mAuto4Note = new Auto4Note(mSwerveSubsystem, mIntakeSubsystem, mShooterSubsystem, mSystemState);
        mAuto2NoteAmpSide = new Auto2NoteAmpSide(mSwerveSubsystem, mIntakeSubsystem, mShooterSubsystem, mSystemState);
        mAuto2NoteCenter = new Auto2NoteCenter(mSwerveSubsystem, mIntakeSubsystem, mShooterSubsystem, mSystemState);
        mJustShoot = new JustShoot(mSystemState, mShooterSubsystem);
        mAuto4Mid = new Auto4RightMidRush(mSwerveSubsystem, mIntakeSubsystem, mShooterSubsystem, mSystemState);
        mAutoLeftMidrush = new AutoLeftMidRush(mSwerveSubsystem, mIntakeSubsystem, mShooterSubsystem, mSystemState);

        //Trigger shoot = new JoystickButton(mDriverController,  XboxController.Button.kRightBumper.value);
        // shoot.whileTrue(new InstantCommand(() -> mSystemState.setSystemState(ESystemState.kShoot)));

        // shoot.whileTrue(new InstantCommand(() -> {
        //     new VisionTurnTo(mSwerveSubsystem, mVisionSubsystem, kAcceleratorHoldVoltage)
        // }));
        //shoot.onFalse(new InstantCommand(() -> mSystemState.setSystemState(ESystemState.kDefault)));

        Trigger intakeToAcceleratorButtton = new Trigger(() -> mDriverController.getLeftTriggerAxis() != 0);
        intakeToAcceleratorButtton.whileTrue(new InstantCommand(() ->{
            if(mSystemState.getRobotState() != ESystemState.kPositionAmp &&
                mArmSubsystem.getTargetWristAngleDegrees() != ArmConstants.kAmpWristDegrees &&
                mArmSubsystem.getTargetShouldAngleDegrees() != ArmConstants.kAmpShoulderDegrees) {
                mSystemState.setSystemState(ESystemState.kIntakeToAccelerator);
            } else {
                mSystemState.setSystemState(ESystemState.kDefault);
            }
        }));
        intakeToAcceleratorButtton.onFalse(new InstantCommand(() -> mSystemState.setSystemState(ESystemState.kDefault)));

        Trigger intakeHoldButtton = new JoystickButton(mDriverController,  XboxController.Button.kLeftBumper.value);
        intakeHoldButtton.whileTrue(new InstantCommand(() -> {
            if(mSystemState.getRobotState() != ESystemState.kPositionAmp &&
                mArmSubsystem.getTargetWristAngleDegrees() != ArmConstants.kAmpWristDegrees &&
                mArmSubsystem.getTargetShouldAngleDegrees() != ArmConstants.kAmpShoulderDegrees) {
                mSystemState.setSystemState(ESystemState.kIntakeAndHold);
            } else {
                mSystemState.setSystemState(ESystemState.kDefault);
            }
        }));
        intakeHoldButtton.onFalse(new InstantCommand(() -> mSystemState.setSystemState(ESystemState.kDefault)));
    
        Trigger ampButton = new Trigger(() -> mDriverController.getPOV() == 0);
        ampButton.whileTrue(new InstantCommand(() -> mSystemState.setSystemState(ESystemState.kPositionAmp)));

        Trigger outtakeButton = new JoystickButton(mDriverController, XboxController.Button.kA.value);
        outtakeButton.whileTrue(new InstantCommand(() -> {
           if (mSystemState.getRobotState() == ESystemState.kPositionAmp) {
                mSystemState.setSystemState(ESystemState.kOuttake);
           }
        }));
        outtakeButton.onFalse(new InstantCommand(() -> {
            if (mSystemState.getRobotState() == ESystemState.kPositionAmp) { 
                mSystemState.setSystemState(ESystemState.kHold);
            }
        }));


        mClimbTeleop = new ClimbTeleop(mClimbSubsystem,
                                       mPartnerController::getXButton,
                                       mPartnerController::getYButton);
        mClimbSubsystem.setDefaultCommand(mClimbTeleop);

        // Swerve commands require a bit more code.
        SlewRateLimiter fwdLimiter = new SlewRateLimiter(SwerveConstants.kDriveSlewRateLimit);
        SlewRateLimiter sideLimiter = new SlewRateLimiter(SwerveConstants.kDriveSlewRateLimit);
        SlewRateLimiter turnLimiter = new SlewRateLimiter(SwerveConstants.kDriveSlewRateLimit);

        mSwerveTeleop = new SwerveTeleop(
            () -> fwdLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriverController.getLeftY())), 
            () -> sideLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriverController.getLeftX())),
            () -> turnLimiter.calculate(OperatorConstants.getControllerProfileValue(-mDriverController.getRightX())),
            mDriverController::getStartButton,
            mSwerveSubsystem);
        mSwerveSubsystem.setDefaultCommand(mSwerveTeleop);

        Trigger shoot = new JoystickButton(mDriverController,  XboxController.Button.kRightBumper.value);

        shoot.whileTrue(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new InstantCommand(() -> mShooterSubsystem.setFlywheelSpeed(kShootingRpm)),
                    new WaitCommand(1.5),
                    new VisionTurnTo(mSwerveSubsystem, mVisionSubsystem, Constants.VisionConstants.kAprilTagTurnTimeout),
                    new InstantCommand(() -> mSystemState.setSystemState(ESystemState.kShoot))
                ),
                new InstantCommand(() -> mSystemState.setSystemState(ESystemState.kShoot)),
                () -> mShooterSubsystem.getShooterMode() == EShooterPivotState.kVisionShot
            )
        );
        shoot.onFalse(new InstantCommand(() -> {
            mSystemState.setSystemState(ESystemState.kDefault);
            mShooterSubsystem.setFlywheelSpeed(kIdleRpm);
        }));


        Trigger enableVisionShot = new JoystickButton(mPartnerController,  XboxController.Button.kA.value);
        enableVisionShot.onTrue(new InstantCommand(() -> mShooterSubsystem.setShooterMode(EShooterPivotState.kVisionShot)));

        Trigger enableCloseShot = new JoystickButton(mPartnerController,  XboxController.Button.kB.value);
        enableCloseShot.onTrue(new InstantCommand(() -> mShooterSubsystem.setShooterMode(EShooterPivotState.kCloseShot)));

        Trigger enableFlywheel = new Trigger(() -> mPartnerController.getLeftTriggerAxis() != 0);
        enableFlywheel.onTrue(new InstantCommand(() -> mShooterSubsystem.setFlywheelSpeed(kShootingRpm)));

        Trigger disableFlywheel = new JoystickButton(mPartnerController,  XboxController.Button.kLeftBumper.value);
        disableFlywheel.onTrue(new InstantCommand(() -> mShooterSubsystem.setFlywheelSpeed(kDisabledRpm)));


        Trigger enableArm = new JoystickButton(mPartnerController,  XboxController.Button.kStart.value);
        enableArm.onTrue(new InstantCommand(() -> {
            mArmSubsystem.setArmStatesEnabled(true);
            mIntakeSubsystem.setEnabled(true);
        }));
        Trigger disableArm = new JoystickButton(mPartnerController,  XboxController.Button.kBack.value);
        disableArm.onTrue(new InstantCommand(() -> {
            mArmSubsystem.setArmStatesEnabled(false);
            mIntakeSubsystem.setEnabled(false);
        }));


        autoSelector.setDefaultOption("Just Shoot", mJustShoot);
        autoSelector.addOption("4 Note", mAuto4Note);
        autoSelector.addOption("2 Note Right", mAuto2NoteAmpSide);
        autoSelector.addOption("2 Note Center", mAuto2NoteCenter);
        autoSelector.addOption("4 Note MidRush", mAuto4Mid);
        autoSelector.addOption("Source Side MidRush", mAutoLeftMidrush);

        driverstationTab.add("Selected Autonomous", autoSelector).withSize(4, 2).withPosition(0, 2).withWidget(BuiltInWidgets.kSplitButtonChooser);

        mLEDs.setState(ELEDState.kVibing);
    }

    public Command getAutonomousCommand() {
        return autoSelector.getSelected();
    }

    public static Pose2d getSpeakerTarget(
        AprilTagFieldLayout pAprilTagFieldLayout) {
        
        Optional<Alliance> alliance = DriverStation.getAlliance();
        Pose2d target = new Pose2d();
        if (alliance.isPresent())
        {
            int aprilTagId = 0;
            
            if(alliance.get() == Alliance.Blue)
            {
                aprilTagId = kBlueSpeaker;
            }
            else if (alliance.get() == Alliance.Red)
            {
                aprilTagId = kRedSpeaker;
            }
            
            Optional<Pose3d> aprilTagOptional = pAprilTagFieldLayout.getTagPose(aprilTagId);
            if (aprilTagOptional.isPresent())
            {
                Pose3d aprilTagPose = aprilTagOptional.get();
                target = new Pose2d(
                    aprilTagPose.getX(), aprilTagPose.getY(),
                        aprilTagPose.getRotation().toRotation2d());
            }
        }
        return target;
    }

    public void setFlywheelSpeed(double pSpeed) {
        mShooterSubsystem.setFlywheelSpeed(pSpeed);
    }
}