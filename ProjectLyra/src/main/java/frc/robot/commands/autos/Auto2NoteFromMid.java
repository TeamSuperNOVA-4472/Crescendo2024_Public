package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SystemState;
import frc.robot.Constants.ESystemState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Auto2NoteFromMid extends ParallelCommandGroup
{
    public Auto2NoteFromMid (SwerveSubsystem pSwerve, IntakeSubsystem pIntake, ShooterSubsystem pShooter, SystemState pSystem)
    {
        addCommands
        (
            new PathCommand("AutoBothFromMid", pSwerve),
            new SequentialCommandGroup
            (
                new InstantCommand
                (() -> 
                    pSystem.setSystemState(ESystemState.kDefault) // Set the robot to its starting position
                ),
                new WaitCommand(2.4),
                new InstantCommand
                (() -> 
                    pSystem.setSystemState(ESystemState.kIntakeAndHold) // Set the robot to its starting position
                ),
                new WaitCommand(1.6),
                new InstantCommand
                (() -> 
                    pSystem.setSystemState(ESystemState.kIntakeToAccelerator) // Set the robot to its starting position
                ),
                new WaitCommand(1.5),
                new InstantCommand
                (() -> 
                    pSystem.setSystemState(ESystemState.kShoot) // Set the robot to its starting position
                ),
                new WaitCommand(2.5),
                new InstantCommand
                (() -> 
                    pSystem.setSystemState(ESystemState.kIntakeAndHold) // Set the robot to its starting position
                ),
                new WaitCommand(1.5),
                new InstantCommand
                (() -> 
                    pSystem.setSystemState(ESystemState.kIntakeToAccelerator) // Set the robot to its starting position
                ),
                new WaitCommand(2),
                new InstantCommand
                (() -> 
                    pSystem.setSystemState(ESystemState.kShoot) // Set the robot to its starting position
                )

            )
        );
    }
}
