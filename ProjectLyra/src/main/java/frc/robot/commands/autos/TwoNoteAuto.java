package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ESystemState;
import frc.robot.SystemState;
import frc.robot.subsystems.SwerveSubsystem;

public class TwoNoteAuto extends ParallelCommandGroup
{
    // Prepares the command for use.
    public TwoNoteAuto(SwerveSubsystem pSwerve, SystemState pSystemState)
    {
        addCommands // Run the "TwoNote" path and turn on the shooter at the same time.
        (
            new PathCommand("TwoNote", pSwerve),
            new SequentialCommandGroup
            (
                new InstantCommand(() ->
                {
                    pSystemState.setSystemState(ESystemState.kShoot);
                }),
                new WaitCommand(2),
                new InstantCommand(() ->
                {
                    pSystemState.setSystemState(ESystemState.kIntakeToAccelerator);
                })
            )
        );
    }
}
