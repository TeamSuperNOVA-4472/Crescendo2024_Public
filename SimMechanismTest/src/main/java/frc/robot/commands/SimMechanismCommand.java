package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimMechanism;

public class SimMechanismCommand extends Command {

    private final SimMechanism mSimMechanism;

    private final DoubleSupplier mElevatorControl;

    private final DoubleSupplier mWristControl;

    public SimMechanismCommand(DoubleSupplier pElevatorControl, DoubleSupplier pWristControl, SimMechanism pSimMechanism) {
        mElevatorControl = pElevatorControl;
        mWristControl = pWristControl;
        mSimMechanism = pSimMechanism;
        addRequirements(mSimMechanism);
    }

    @Override
    public void execute() {
        mSimMechanism.setMechanismState(mElevatorControl.getAsDouble(), mWristControl.getAsDouble());
    }
    
}
