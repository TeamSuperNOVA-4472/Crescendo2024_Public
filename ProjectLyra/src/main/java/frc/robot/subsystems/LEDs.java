package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.ELEDState;

import static frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase
{
    private final Spark mLightingDriver1;
    private final Spark mLightingDriver2;

    private ELEDState mCurrentState;

    public LEDs()
    {
        mLightingDriver1 = new Spark(LEDConstants.kBlinkinPWMPort1);
        mLightingDriver2 = new Spark(LEDConstants.kBlinkinPWMPort2);

        mCurrentState = ELEDState.kVibing;
    }

    public void setState(ELEDState pNewMode)
    {
        mCurrentState = pNewMode;
    }

    private void processVibing()
    {
      mLightingDriver1.set(LEDConstants.kVibingSignal);
      mLightingDriver2.set(LEDConstants.kVibingSignal);
    }

    private void processInIntake()
    {
      mLightingDriver1.set(LEDConstants.kNoteInIntakeSignal);
      mLightingDriver2.set(LEDConstants.kNoteInIntakeSignal);
    }

    private void processInAccel()
    {
      mLightingDriver1.set(LEDConstants.kNoteInAcceleratorSignal);
      mLightingDriver2.set(LEDConstants.kNoteInAcceleratorSignal);
    }

    private void processShooting()
    {
      mLightingDriver1.set(LEDConstants.kShootingSignal);
      mLightingDriver2.set(LEDConstants.kShootingSignal);
    }

    private void processFailure()
    {
      mLightingDriver1.set(LEDConstants.kFailureSignal);
      mLightingDriver2.set(LEDConstants.kFailureSignal);
    }



    @Override
    public void periodic()
    {
        switch (mCurrentState)
        {
            case kVibing:
              processVibing();
              break;

            case kNoteInIntake:
              processInIntake();
              break;

            case kNoteInAccelerator:
              processInAccel();
              break;

            case kShooting:
              processShooting();
              break;

            case kFailure:
              processFailure();
              break;


            default: return; // Unknown lighting mode.
        }
    }
}