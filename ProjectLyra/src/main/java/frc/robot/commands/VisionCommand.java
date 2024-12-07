package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.VisionSubsystem;

// This is unfinished. Does literally nothing.
public class VisionCommand extends Command
{   
    // The subsystem used by this command.
    private final VisionSubsystem mVisionSubsystem;

    // Prepares the command for use.
    public VisionCommand(VisionSubsystem pVisionSubsystem)
    {
        // Add a requirement for the vision subsystem.
        mVisionSubsystem = pVisionSubsystem;
        addRequirements(mVisionSubsystem);
    }    
}
