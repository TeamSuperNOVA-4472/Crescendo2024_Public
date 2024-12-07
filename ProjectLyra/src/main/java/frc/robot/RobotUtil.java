package frc.robot;

/**
 * Utility class containing commonly used methods on the robot
 */
public final class RobotUtil {

    public static double distanceFromTarget(double pMeasured, double pTarget) {
        return Math.abs(pMeasured - pTarget);
    }

    private RobotUtil() {
        // Do not instantialte
    }
}
