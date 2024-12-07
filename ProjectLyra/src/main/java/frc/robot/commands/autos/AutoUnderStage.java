package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants.EIntakeState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// An autonomous command that sends the robot underneath the stage.
public class AutoUnderStage extends ParallelCommandGroup
{
    // Prepares the auto command for use.
    public AutoUnderStage(SwerveSubsystem pSwerve, IntakeSubsystem pIntake, ShooterSubsystem pShooter)
    {
        // Two commands are run at the same time: the path follower and the actual
        // subsystem control.

        addCommands // Do the following at the same time:
        (
            new SequentialCommandGroup // Do the following one-after-another:
            (
                new PathCommand("Auto Under Stage Part 1", pSwerve), // Play part 1.
                new WaitCommand(1),                                // Wait 1 second.
                new PathCommand("Auto Under Stage Part 2", pSwerve)  // Play part 2.
            ),
            new SequentialCommandGroup // Do the following one-after-another:
            (
                new InstantCommand(() ->      // Set the flywheel to 1000 RPM and 30 degrees up.
                {
                    pShooter.setFlywheelSpeed(1000);
                    pShooter.setAngleDegrees(30);
                }),
                new WaitCommand(1.1), // Wait 1.1 seconds.

                new InstantCommand(() ->      // Turn the intake on.
                {
                    //pIntake.setState(EIntakeState.kIntake);
                }),
                new WaitCommand(0.5), // Wait 0.5 seconds.

                new InstantCommand(() ->      // Turn the flywheel off.
                {
                    pShooter.setFlywheelSpeed(0);
                }),
                new WaitCommand(1.6), // Wait 1.6 seconds.

                new InstantCommand(() ->      // Turn off the intake.
                {
                    //pIntake.setState(EIntakeState.kStop);
                }),
                new WaitCommand(1.3), // Wait 1.3 seconds.

                new InstantCommand(() ->      // Turn the flywheel back on and move it 5 degrees down.
                {
                    pShooter.setFlywheelSpeed(1000);
                    pShooter.setAngleDegrees(25);
                }),
                new WaitCommand(0.9), // Wait 0.9 seconds.

                new InstantCommand(() ->      // Turn the intake on.
                {
                    //pIntake.setState(EIntakeState.kIntake);
                }),
                new WaitCommand(0.7), // Wait 0.7 seconds.

                new InstantCommand(() ->      // Turn the flywheel off.
                {
                    pShooter.setFlywheelSpeed(0);
                }),
                new WaitCommand(3.4), // Wait 3.4 seconds.

                new InstantCommand(() ->      // Turn the intake off.
                {
                    //pIntake.setState(EIntakeState.kStop);
                }),
                new WaitCommand(0.5), // Wait 0.5 seconds.

                new InstantCommand(() ->      // Turn the flywheel back on (where it is).
                {
                    pShooter.setFlywheelSpeed(1000);
                }),
                new WaitCommand(1),   // Wait 1 second.

                new InstantCommand(() ->      // Turn the intake on.
                {
                    //pIntake.setState(EIntakeState.kIntake);
                }),
                new WaitCommand(1),   // Wait 1 second.

                new InstantCommand(() ->      // Turn everything off. End it.
                {
                    pShooter.setFlywheelSpeed(0);
                    pShooter.setAngleDegrees(0);
                    //pIntake.setState(EIntakeState.kStop);
                })
            )
        );

        // Add requirements to each subsystem used.
        addRequirements(pSwerve, pIntake, pShooter);
    }
}
