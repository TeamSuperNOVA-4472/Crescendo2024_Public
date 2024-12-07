package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimDrive;

public class SimDriveCommand extends Command {

    private final DoubleSupplier mMagSpeedX;
    private final DoubleSupplier mMagTurnZ;
    private final SimDrive mSimDrive;

    public SimDriveCommand(SimDrive pSimDrive, DoubleSupplier pMagSpeedX, DoubleSupplier pMagTurnZ) {
        mMagSpeedX = pMagSpeedX;
        mMagTurnZ = pMagTurnZ;
        mSimDrive = pSimDrive;
        addRequirements(pSimDrive);
    }

    @Override
    public void execute() {
        mSimDrive.driveArcade(mMagSpeedX.getAsDouble(), mMagTurnZ.getAsDouble());
    }
    
}
