package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

// This command gives the user control of the arm.
public class ArmDefault extends Command
{
    
    // The arm subsystem that we control.
    private final ArmSubsystem mArmSubsystem;

    private final Supplier<Double> mShoulderSupplier;
    private final Supplier<Double> mWristSupplier;


    // Prepares the command for use.
    public ArmDefault(ArmSubsystem pArmSubsystem, Supplier<Double> pShoulderSupplier, Supplier<Double> pWristSupplier)
    {
        mArmSubsystem = pArmSubsystem;
        mShoulderSupplier = pShoulderSupplier;
        mWristSupplier = pWristSupplier;
        addRequirements(mArmSubsystem);
    }

    @Override
    public void initialize() {
        mArmSubsystem.reset();
    } 

    // Called over and over again.
    @Override
    public void execute()
    {
        if(!mArmSubsystem.isArmStatesEnabled()) {
            mArmSubsystem.setRawShoulderPercent(mShoulderSupplier.get());
            mArmSubsystem.setRawWristPercent(mWristSupplier.get());
        }
    }
}
