package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// This is where the big stuff happens. We create and use subsystems and commands here.
public class RobotContainer
{
    // Declare the subsystems and commands that are going to be used.
    private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // Declare the controller used. Could also be a `CommandPS4Controller` or `CommandJoystick`
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    // Triggered on robot start.
    public RobotContainer()
    {
        // Configure the bindings.
        configureBindings();
    }

    // Configure the trigger bindings for the controller.
    private void configureBindings()
    {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(m_exampleSubsystem::exampleCondition)
            .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Set the `exampleMethodCommand` to run when the B button is pressed.
        // When B is let go, the command is cancelled.
        m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    }

    // Used to choose an autonomous command.
    public Command getAutonomousCommand()
    {
        // An example command will be run in autonomous.
        return Autos.exampleAuto(m_exampleSubsystem);
    }
}
