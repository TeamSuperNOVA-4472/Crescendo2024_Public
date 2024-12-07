package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDefault extends Command
{
    
    // The arm subsystem that we control.
    private final IntakeSubsystem mIntakeSubsystem;

    private final Supplier<Boolean> mRunIntake;

    private final Supplier<Boolean> mRunOuttake;


    // Prepares the command for use.
    public IntakeDefault(IntakeSubsystem pIntakeSubsystem, Supplier<Boolean> pRunIntake, Supplier<Boolean> pRunOuttake)
    {
        mIntakeSubsystem = pIntakeSubsystem;
        mRunIntake = pRunIntake;
        mRunOuttake = pRunOuttake;
        addRequirements(mIntakeSubsystem);
    }


    // // Called over and over again.
    @Override
    public void execute()
    {
        if(!mIntakeSubsystem.isEnabled()) {
            if(mRunIntake.get()) {
                mIntakeSubsystem.setIntakeVoltage(-IntakeConstants.kIntakeVoltage);
            } 
            else if(mRunOuttake.get()) {
                mIntakeSubsystem.setIntakeVoltage(IntakeConstants.kIntakeVoltage);
            }
            else {
                mIntakeSubsystem.setIntakeVoltage(0);
            }
        }
    }
}
