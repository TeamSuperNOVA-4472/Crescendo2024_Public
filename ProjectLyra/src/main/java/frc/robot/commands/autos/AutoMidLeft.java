package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.SystemState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.*;
public class AutoMidLeft extends ParallelCommandGroup 
{
    public AutoMidLeft (SwerveSubsystem pSwerve, IntakeSubsystem pIntake, ShooterSubsystem pShooter, SystemState pSystem)
    {
        addCommands
        (
            new PathCommand("TwoNoteMidRush", pSwerve),
            new SequentialCommandGroup
            (
                new InstantCommand
                (() ->
                    pSystem.setSystemState(ESystemState.kDefault)
                ),
                new WaitCommand(1),
                new InstantCommand
                (() ->
                    pSystem.setSystemState(ESystemState.kIntakeAndHold)
                ),
                new WaitCommand(3.8),
                new InstantCommand
                (() ->
                    pSystem.setSystemState(ESystemState.kIntakeToAccelerator)
                ),
                new WaitCommand(4.2),
                new InstantCommand
                (() ->
                    pSystem.setSystemState(ESystemState.kShoot)
                ),
                new WaitCommand(.6),
                new InstantCommand
                (() ->
                    pSystem.setSystemState(ESystemState.kIntakeAndHold)
                ),
                new WaitCommand(1.5),
                new InstantCommand
                (() ->
                    pSystem.setSystemState(ESystemState.kIntakeToAccelerator)
                ),
                new WaitCommand(.5),
                new InstantCommand
                (() ->
                    pSystem.setSystemState(ESystemState.kShoot)
                )
            )
        );
    }
}
