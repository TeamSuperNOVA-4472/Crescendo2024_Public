package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

// This file stores the constants that are used throughout the robot.
// It's stored in one place so that we don't have to modify many values
// all over the place.
public final class Constants
{

    public static class CommonConstants {
        public static final double kMaxVolts = 12.0;
        public static final double kDegreesTolerance = 5;
        public static final String kCanivoreName = "canivore";
    }

    public enum ESystemState {
        kDefault,
        kHold,
        kIntakeAndHold,
        kIntakeToAccelerator,
        kPositionAmp,
        kPositionTrap,
        kOuttake,
        kOuttakeFloor,
        kShoot,
        kClimb;
    }

    // Constants relating to the intake subsystem.
    public static class IntakeConstants
    {

        public enum EIntakeState {
            kStop,
            kIntake,
            kFeedAccelerator,
            kOuttake,
            kHoldingNote;
        }

        // Intake motor CAN ID.
        public static final int kIntakeId = 10;

        // Note detector RIO port.
        public static final int kIntakeNoteDetectorPort = 3;

        // The voltage used when the intake is enabled.
        public static final double kIntakeVoltage = 6.15;

        public static final double kOutakeVoltage = 8.0;

        public static final double kAdditionalDistance = 4.05;
    }

    // Constants relating to the intake subsystem.
    public static class ClimbConstants
    {
        // Intake motor CAN ID.
        public static final int kClimbLeftId = 21;
        public static final int kClimbRightId = 20;

        // The voltage used when the intake is enabled.
        public static final double kClimbVoltage = 6.0;
    
        // The limits for the climber to prevent going too far.
        public static final double kClimberLeftUpperLimit = 100;
        public static final double kClimberRightUpperLimit = 100;

        public static final double kClimbG = 0.015;

    }

    // Constants relating to the shooter subsystem.
    public static class ShooterConstants
    {
        public enum EShooterPivotState {
            kIntakeTransfer,
            kCloseShot,
            kVisionShot,
            kTransition,
            kClimb;
        }

        public enum EAcceleratorState {
            kAcceleratorHold,
            kToShooter,
            kToIntake,
            kHoldingNote,
            kStop;
        }

        // The flywheel and accelerator motor ports.
        public static final int kFlywheelTopId = 9;
        public static final int kFlywheelBotId = 8;
        public static final int kAcceleratorId = 38;

        // The flywheel angler and its encoder's ports.
        public static final int kAngleMotorId = 35;
        public static final int kAngleEncoderId = 0;

        // The flywheel PIDF
        public static final double kFlywheelS = 0.05;
        public static final double kFlywheelV = 0.12;
        public static final double kFlywheelP = 0.11;
        public static final double kShootingRpm = 4000;
        public static final double kDisabledRpm = 600;
        public static final double kRpsToRpm = 60;
        public static final double kIdleRpm = 1250;

        // The flywheel angler PID.
        public static final double kAngleP = 0.018;
        public static final double kAngleI = 0;
        public static final double kAngleD = 0.0005;
        public static final double kAngleG = 0.025;

        // The distance range allowed to make a shot.
        public static final double kMinDistance = 0;
        public static final double kMaxDistance = 100;

        // At the minimum and maximum distances, what flywheel speeds
        // are required to make the shot?
        public static final double kMinFlywheelRPM = 0;
        public static final double kMaxFlywheelRPM = 6000;

        // At the minimum and maximum distances, what is the angle
        // required for the flywheel to make the shot?
        public static final double kMinAngleDegrees = 30;
        public static final double kMaxAngleDegrees = 70;

        // April tag IDs for the red and blue speaker tags
        public static final int kBlueSpeaker = 7;
        public static final int kRedSpeaker = 4;

        // Note detector RIO port.
        public static final int kAcceleratorNoteDetectorPort = 4;
        // The default accelerator voltage to apply
        public static final double kAcceleratorHoldVoltage = 7.0;
        public static final double kAcceleratorShootVolatage = 12.0;

        public static final double kSecondsPerMin = 60;

        public static final double kPivotRotation = 360.0 * 24/77;

        public static final double kPivotOffset = -2 - 5 - 6;
        public static final double kPivotNormalizeDegrees = 90;

        // The angle of the pivot to take a note into the accelerator
        public static final double kPivotIntakeDegrees = -6;
        public static final double kPivotCloseDegrees = 55;

        // Climb
        public static final double kPivotClimbDegrees = 20;
    }

    // Constants relating to the arm subsystem.
    public static class ArmConstants
    {

        // The possible states for the arm to have.
        public enum EArmState {
            kAcceleratorTransfer, // Default position, which is to positioned transfer to accelerator
            kIntake, // Positioned to intake a note.
            kAmp, // Positioned to place a note in the amplifier.
            kTrap, // Positioned to place a note in the trap
            kTransition;
        }

        // Wrist motor and encoder IDs.
        public static final int kWristRightId = 33;
        public static final int kWristLeftId = 34;

        // Shoulder motor and encoder IDs.
        public static final int kShoulderLeftId = 36;
        public static final int kShoulderRightId = 37;
        public static final int kLeftShoulderEncoderId = 2;
        public static final int kRightShoulderEncoderId = 1;

        // Wrist PID values.
        public static final double kWristP = 0.0035;
        public static final double kWristI = 0;
        public static final double kWristD = 0.0001;
        public static final double kWristG = 0.015;
        public static final double kWristRamp = 0.25;

        // Wrist angles
        public static final double kDefaultWristDegrees = 7;
        public static final double kIntakeWristDegrees = 200;
        public static final double kWristNormalizedDegrees = 250;
        public static final double kAmpWristDegrees = 100;
        
        public static final double kRightWristOffset = 33;
        public static final double kLeftWristOffset = 302;

        // Shoulder PID values.
        public static final double kShoulderP = 0.02;
        public static final double kShoulderI = 0;
        public static final double kShoulderD = 0.001;
        public static final double kShoulderG = 0.04;
        public static final double kShoulderExtG = 0.01;
        public static final double kShoulderRamp = 0.35;
        public static final double kShoulderNormalizedDegrees = 270;
        public static final double kMaxShoulderSpeedDegSec = 360;
        public static final double kMaxShoulderAccelDegSec = 360;
        public static final double kMaxWristSpeedDegSec = 2880;
        public static final double kMaxWristAccelDegSec = 2880;

        public static final double kRotation = 360;
        public static final double kLeftShoulderOffset = 123;
        public static final double kRightShoulderOffset = 71;

        public static final double kDefaultShoulderDegrees = -30;
        public static final double kAmpShoulderDegrees = 30;

        // The possible states for the arm to have.
        public static enum ArmState
        {
            kDefault, // Default position.
            kIntake,  // Positioned to intake a note.
            kAmp;     // Positioned to place a note in the amplifier.
        }
    }

    // Constants relating to the vision subsystem.
    public static class VisionConstants
    {
        // The names of each camera used.
        // public static final String kFrontCam = "Arducam_OV9281_USB_Camera";
        public static final String kBackCam = "Global_Shutter_Camera";

        // The position/rotation of each camera used.
        public static final Transform3d kFrontCamLoc = new Transform3d();
        public static final Transform3d kBackCamLoc = new Transform3d();

        public static final double kAprilTagOffsetDegs = 0.0;

        // This could use either 2D or 3D AprilTag processing, I don't know which one would be better though 🤔
        public static final InterpolatingDoubleTreeMap kPivotMap = new InterpolatingDoubleTreeMap();
        
        static {
            // these are pitch values for the tag on photonvision
            kPivotMap.put(8.32, 55.0);
            kPivotMap.put(2.02, 50.0);
            kPivotMap.put(0.78, 45.0);
            kPivotMap.put(-2.09, 40.0);
            kPivotMap.put(-5.78, 36.0);
            kPivotMap.put(-9.83, 31.5);
            kPivotMap.put(-13.6, 21.65);
        }

        public static final InterpolatingDoubleTreeMap kFlywheelTargetSpeedMap = new InterpolatingDoubleTreeMap();

        static {

        }

        public static final double kAprilTagTurnTimeout = 1.0; // This is in seconds
        public static final double kTagAlignmentErrDeg = 2.25; 
    }

    // Constants relating to the driver and the operator.
    public static class OperatorConstants
    {
        // The port of the controller used by the driver.
        public static final int kDriverControllerPort = 0;
        public static final int kPartnerControllerPort = 1;
        public static final double kDeadband = 0.15;
        public static final InterpolatingDoubleTreeMap kControllerProfileMap = new InterpolatingDoubleTreeMap();

        static // This is a static constructor. Used to assign values to static variables.
        {
            // kControllerProfileMap.put(0.0, 0.0);
            // kControllerProfileMap.put(0.8, 1.0/3.0);
            // kControllerProfileMap.put(1.0, 1.0);

            kControllerProfileMap.put(0.0, 0.0);
            kControllerProfileMap.put(0.663, 1.0/2.5);
            kControllerProfileMap.put(1.0, 1.0);
        }

        public static double getControllerProfileValue(double pValue)
        {
            double deadbandedVal = MathUtil.applyDeadband(pValue, kDeadband);
            double clampedVal = MathUtil.clamp(deadbandedVal, -1, 1);
            return Math.signum(clampedVal) * kControllerProfileMap.get(Math.abs(clampedVal));
        }
    }

    // Constants regarding the swerve drive modules.
    public static class SwerveConstants
    {
        public static final double kMaxSpeedMS = 5.4;
        public static final double kMetersPerInch = Units.inchesToMeters(1);

        public static final double kSwerveLocXInches = 12.375;
        public static final double kSwerveLocYInches = 11.375;

        public static final double kSwerveLocXMeters = kSwerveLocXInches * kMetersPerInch;
        public static final double kSwerveLocYMeters = kSwerveLocYInches * kMetersPerInch;

        public static final double kSwerveRadiusInches = Math.sqrt(Math.pow(kSwerveLocXInches, 2) + Math.pow(kSwerveLocYInches, 2));
        public static final double kSwerveCircumferenceMeters = 2 * Math.PI * kSwerveRadiusInches * kMetersPerInch; //1.6372863352652361048052029816421;

        public static final double kMetersPerSecondToRadiansPerSecond = (2 * Math.PI) / kSwerveCircumferenceMeters;
        public static final double kMetersPerSecondToDegreesPerSecond = (360.0) / kSwerveCircumferenceMeters;

        public static final double kS = .067;
        public static final double kV = 2.20;
        public static final double kA = 0.23;

        public static final double kDriveSlewRateLimit = 3.0;
    }

    public static class LEDConstants {
        public static final int kBlinkinPWMPort1 = 8;
        public static final int kBlinkinPWMPort2 = 9;

        public static final double kVibingSignal = -0.55;
        public static final double kNoteInIntakeSignal = -0.07;
        public static final double kNoteInAcceleratorSignal = -0.45;
        public static final double kShootingSignal = -0.05;
        public static final double kFailureSignal = -0.11;

        public enum ELEDState {
            kVibing,
            kNoteInIntake,
            kNoteInAccelerator,
            kShooting,
            kFailure;
        }
    }
}
