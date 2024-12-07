package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// The list of autonomous commands that could be run.
public final class Autos
{
    // A static factor for an example autonomous command.
    public static Command exampleAuto(ExampleSubsystem subsystem)
    {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }

    private Autos()
    {
        // Doesn't do anything, since you aren't supposed to construct this object.
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
