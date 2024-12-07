package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ExampleCommand;

// An example subsystem.
public class ExampleSubsystem extends SubsystemBase
{
    // Creates a new instance of the example subsystem.
    public ExampleSubsystem() { }

    // Example command factory. Constructs a command on demand.
    public Command exampleMethodCommand()
    {
        return new ExampleCommand(this);
    }

    // An example condition that can be checked.
    public boolean exampleCondition()
    {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    // Run over and over again every time the scheduler is run.
    @Override
    public void periodic() { }

    // Same as `periodic()` but only run when loaded in a simulation.
    @Override
    public void simulationPeriodic() { }
}
