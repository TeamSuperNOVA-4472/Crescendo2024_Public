package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ESystemState;
import frc.robot.Constants.ShooterConstants.EShooterPivotState;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.SystemState;

public class JustShoot extends SequentialCommandGroup {
    public JustShoot(SystemState pSystemState, ShooterSubsystem pShooterSubsystem) {
        addCommands(
            new WaitCommand(3.0),
            new InstantCommand(() -> {
                pSystemState.setSystemState(ESystemState.kShoot);
                pShooterSubsystem.setShooterMode(EShooterPivotState.kCloseShot);
            }),
            new WaitCommand(3.0),
            new InstantCommand(() -> {
                pSystemState.setSystemState(ESystemState.kDefault);
                pShooterSubsystem.setShooterMode(EShooterPivotState.kVisionShot);
            })
        );
    }
}
