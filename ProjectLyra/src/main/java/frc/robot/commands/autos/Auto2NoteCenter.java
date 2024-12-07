package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ESystemState;
import frc.robot.SystemState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto2NoteCenter extends ParallelCommandGroup{
    // Prepares the auto command for use.
public Auto2NoteCenter(SwerveSubsystem pSwerve, IntakeSubsystem pIntake, ShooterSubsystem pShooter, SystemState pSystem)
{
    // Two commands are run at the same time: the path follower and the actual
    // subsystem control.

    addCommands // Do the following at the same time:
    (
        new PathCommand("2 note center", pSwerve), // Play the main path.
        new SequentialCommandGroup // Do the following one-after-another:
        (
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kDefault) // Set the robot to its starting position
            ),
            new WaitCommand(1.9), // Wait 1.9 seconds
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kShoot)  // Fire a note 
            ),
            new WaitCommand(0.4), // Wait 0.4 seconds
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kIntakeAndHold) // Prepare to grab a note
            ),
            new WaitCommand(2.5), // Wait 2.5 seconds
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kIntakeToAccelerator) // Put the note into the accelerator
            ),
            new WaitCommand(2.5), // Wait 2.5 seconds
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kShoot) // Fire a note
            ),
            new WaitCommand(1), // Wait 1 second
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kDefault) // Set the robot to its starting position
            )
        )
    );
    // Add requirements to each subsystem used.
    addRequirements(pSwerve, pIntake, pShooter);
}
}
