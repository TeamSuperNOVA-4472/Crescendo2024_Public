package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimMechanism extends SubsystemBase {

    private static final double ELEVATOR_DEFAULT_HEIGHT = 1.5;
    private static final double ELEVATOR_DEFAULT_ANGLE_DEG = 90.0;
    private static final double WRIST_DEFAULT_LENGTH = 1.5;
    private static final double WRIST_DEFAULT_ANGLE = 90;

    private final Mechanism2d mMechanism = new Mechanism2d(6, 6);

    private final MechanismRoot2d mRoot = mMechanism.getRoot("Sim Root", 1.5, 0);

    private final MechanismLigament2d mElevator = new MechanismLigament2d("elevator", ELEVATOR_DEFAULT_HEIGHT, ELEVATOR_DEFAULT_ANGLE_DEG);
    private final MechanismLigament2d mWrist = new MechanismLigament2d("wrist", 0, 0);

    public SimMechanism() {
        mRoot.append(mElevator);
        mElevator.append(mWrist);
        mWrist.setAngle(-WRIST_DEFAULT_ANGLE);
        mWrist.setLength(WRIST_DEFAULT_LENGTH);
        SmartDashboard.putData("Sim Mechanism", mMechanism);
    }

    public void setMechanismState(double pElevatorControl, double pWristControl) {
        mElevator.setLength(pElevatorControl * ELEVATOR_DEFAULT_HEIGHT + ELEVATOR_DEFAULT_HEIGHT);
        mWrist.setAngle(pWristControl * WRIST_DEFAULT_ANGLE - WRIST_DEFAULT_ANGLE);
    }
    
}
