package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

// An example command.
public class ExampleCommand extends Command
{
    // Currently unused. Would be used in the future to talk to
    // other things the subsystem controls.
    private final ExampleSubsystem m_subsystem;

   // Creates a new instance of this example command.
    public ExampleCommand(ExampleSubsystem subsystem)
    {
        m_subsystem = subsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is first loaded. Only called once, even if the command
    // is enabled several times.
    @Override
    public void initialize() { }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
