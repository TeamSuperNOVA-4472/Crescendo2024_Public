package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

// This main class simply calls the RobotContainer class and the RobotBase class.
// We don't want to put any other code here.
public final class Main
{
    private Main() {}

    public static void main(String... args)
    {
        RobotBase.startRobot(Robot::new);
    }
}
