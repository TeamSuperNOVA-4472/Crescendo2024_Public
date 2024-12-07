package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimbSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.ClimbConstants.*;

// The teleop command for the climb subsystem.
public class ClimbTeleop extends Command
{
    // The subsystem this command is intended for.
    private final ClimbSubsystem mClimbSubsystem;

    // The climb up and climb down buttons. It's a supplier because we don't
    // need to know the whole controller state here, just those two buttons.
    private Supplier<Boolean> mClimbUpSupplier;
    private Supplier<Boolean> mClimbDownSupplier;

    // Initializes the command.
    public ClimbTeleop(
        ClimbSubsystem pClimbSubsystem,
        Supplier<Boolean> pClimbUpSupplier,
        Supplier<Boolean> pClimbDownSupplier)
    {
        // Add subsystem requirements.
        addRequirements(pClimbSubsystem);

        // Assign values.
        mClimbSubsystem = pClimbSubsystem;
        mClimbUpSupplier = pClimbUpSupplier;
        mClimbDownSupplier = pClimbDownSupplier;
    }

    // Called when the command is first started.
    @Override
    public void initialize() { }

    // Called over and over again as long as the command is running.
    @Override
    public void execute()
    {
        if (mClimbUpSupplier.get()) // If the climb up button is pressed, move climb winch upwards.
        {
            mClimbSubsystem.setVoltage(kClimbVoltage);
        }
        else if (mClimbDownSupplier.get()) // If the climb down button is pressed, move winch climb downwards.
        {
            mClimbSubsystem.setVoltage(-kClimbVoltage - kClimbG);
        }
        else // If neither, stop the climb.
        {
            mClimbSubsystem.setVoltage(0.0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false; // Teleop never finishes.
    }
}
