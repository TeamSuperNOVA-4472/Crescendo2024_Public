package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// This class contains the root of all the code we want to run. Different code can be run depending
// on the state of the robot, whether it's in autonomouys or teleop among others.
public class Robot extends TimedRobot
{
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    // Run when the robot first starts.
    @Override
    public void robotInit()
    {
        // Create our new container. The container controls button mappings among other
        // things.
        m_robotContainer = new RobotContainer();
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
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Schedule the autonomous command if it exists.
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.schedule();
        }
    }

    // Run over and over again as long as the robot is in autononmous.
    @Override
    public void autonomousPeriodic() { }

    // Run ONCE when the robot first enters teleop.
    @Override
    public void teleopInit()
    {
        if (m_autonomousCommand != null)
        {
            m_autonomousCommand.cancel();
        }
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
