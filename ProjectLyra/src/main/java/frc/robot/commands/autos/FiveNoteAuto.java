package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class FiveNoteAuto extends ParallelCommandGroup {
    public FiveNoteAuto(SwerveSubsystem pSwerveSubsystem, ShooterSubsystem pShooterSubsystem, IntakeSubsystem pIntakeSubsystem, ArmSubsystem pArmSubsystem) {
        addCommands(
            new PathCommand("FiveNoteAuto", pSwerveSubsystem),
            new SequentialCommandGroup(
                new InstantCommand(() -> {
                    pShooterSubsystem.setAngleDegrees(30);
                    pShooterSubsystem.setFlywheelSpeed(1000);
                })
            )
        );
        addRequirements(pShooterSubsystem, pIntakeSubsystem, pArmSubsystem);
    }
}
