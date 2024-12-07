package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.Constants.CommonConstants;

import static frc.robot.Constants.ClimbConstants.*;

// This subsystem controls the climbing winch.
public class ClimbSubsystem extends SubsystemBase
{
    // The physical motor. Will be set to `null` when in simulation.
    private final TalonFX mClimbLeftMotor;
    private final TalonFX mClimbRightMotor;

    // The current voltage.
    private double mCurrentVoltage = 0;

    // Creates the climb subsystem.
    public ClimbSubsystem()
    {
        if (Robot.isReal())
        {
            mClimbLeftMotor = new TalonFX(kClimbLeftId, CommonConstants.kCanivoreName);
            mClimbRightMotor = new TalonFX(kClimbRightId, CommonConstants.kCanivoreName);
        }
        else
        {
            // In simulation, there are no real motors, so we don't create them.
            mClimbLeftMotor = null;
            mClimbRightMotor = null;
        }
    }

    // Update the climb voltage.
    public void setVoltage(double pNewVoltage)
    {
        mCurrentVoltage = pNewVoltage;

        if (Robot.isReal())
        {
            mClimbLeftMotor.setVoltage(-mCurrentVoltage);
            mClimbRightMotor.setVoltage(mCurrentVoltage);
        }
    }

    // Get the current climb voltage.
    public double getVoltage()
    {
        return mCurrentVoltage;
    }

    // Called over and over again.
    @Override
    public void periodic()
    {
        // Update the dashboard with our current voltage.
        SmartDashboard.putNumber("Climb Voltage", mCurrentVoltage);
    }
}
