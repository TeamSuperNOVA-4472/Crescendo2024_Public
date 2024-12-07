package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SimDrive;

public class SimDriveSwerveCommand extends Command {
    private final DoubleSupplier mMagSpeedX;
    private final DoubleSupplier mMagSpeedY;
    private final DoubleSupplier mMagTurnZ;
    private final SimDrive mSimDrive;

    public SimDriveSwerveCommand(SimDrive pSimDrive, DoubleSupplier pMagSpeedX, DoubleSupplier pMagSpeedY, DoubleSupplier pMagTurnZ) {
        mSimDrive = pSimDrive;
        mMagSpeedX = pMagSpeedX;
        mMagSpeedY = pMagSpeedY;
        mMagTurnZ = pMagTurnZ;
        addRequirements(pSimDrive);
    }

    @Override
    public void execute() {
        mSimDrive.driveFieldOrientedSwerve(mMagSpeedX.getAsDouble(), mMagSpeedY.getAsDouble(), mMagTurnZ.getAsDouble());
    }
}
