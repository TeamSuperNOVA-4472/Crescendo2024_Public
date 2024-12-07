package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.SystemState;
import frc.robot.Constants.*;

public class Auto2NoteAmpSide extends ParallelCommandGroup{
// Prepares the auto command for use.
public Auto2NoteAmpSide(SwerveSubsystem pSwerve, IntakeSubsystem pIntake, ShooterSubsystem pShooter, SystemState pSystem)
{
    // Two commands are run at the same time: the path follower and the actual
    // subsystem control.

    addCommands // Do the following at the same time:
    (
        new PathCommand("Auto 2note closeAmp", pSwerve), // Play the main path.
        new SequentialCommandGroup // Do the following one-after-another:
        (
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kDefault) // Set the robot to its starting position
            ),
            new WaitCommand(1.4), // Wait 1.4 seconds
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kShoot)  // Fire a note 
            ),
            new WaitCommand(0.5), // Wait 0.5 seconds
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kIntakeAndHold) // Prepare to grab a note
            ),
            new WaitCommand(2.3), // Wait 2.3 seconds
            new InstantCommand
            (() -> 
                pSystem.setSystemState(ESystemState.kIntakeToAccelerator) // Put the note into the accelerator
            ),
            new WaitCommand(2), // Wait 2 seconds
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