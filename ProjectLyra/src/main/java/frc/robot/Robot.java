package frc.robot;

import static frc.robot.Constants.ShooterConstants.kIdleRpm;
import static frc.robot.Constants.ShooterConstants.kShootingRpm;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// This class contains the root of all code we want to run. Different code can be run depending
// on the state of the robot, whether it's in autonomous or teleop for example.
public class Robot extends TimedRobot
{
    // The chosen autonomous command.
    private Command mAutonomousCommand;

    // The container for the robot.
    private RobotContainer mRobotContainer;

    // Run when the robot first starts.
    @Override
    public void robotInit()
    {
        // Create our new container. The container controls button mappings among other
        // things.
        mRobotContainer = new RobotContainer();
    }

    // This is constantly run over and over again while the robot is active.
    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();
    }

    // Run ONCE when the robot first is disabled.
    @Override
    public void disabledInit() { }

    // Run over and over again as long as the robot is disabled.
    @Override
    public void disabledPeriodic() { }

    // Run ONCE when the robot first enters autonomous.
    @Override
    public void autonomousInit()
    {
        mAutonomousCommand = mRobotContainer.getAutonomousCommand();
        mRobotContainer.setFlywheelSpeed(kShootingRpm);
        // Schedule the autonomous command if it exists.
        if (mAutonomousCommand != null)
        {
            mAutonomousCommand.schedule();
        }
    }

    // Run over and over again as long as the robot is in autononmous.
    @Override
    public void autonomousPeriodic() { }

    // Run ONCE when the robot first enters teleop.
    @Override
    public void teleopInit()
    {
        if (mAutonomousCommand != null)
        {
            mAutonomousCommand.cancel();
        }
        mRobotContainer.setFlywheelSpeed(kIdleRpm);
    }

    // Run over and over again as long as the robot is in teleop.
    @Override
    public void teleopPeriodic() { }

    // Run ONCE when the robot enters test mode.
    @Override
    public void testInit()
    {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    // Run over and over again as long as the robot is in test mode.
    @Override
    public void testPeriodic() { }

    // Run ONCE when the simulated robot starts. (Simulation only)
    @Override
    public void simulationInit() { }

    // Run over and over again as long as the robot is in simulation mode. (Simulator only)
    @Override
    public void simulationPeriodic() { }
}
