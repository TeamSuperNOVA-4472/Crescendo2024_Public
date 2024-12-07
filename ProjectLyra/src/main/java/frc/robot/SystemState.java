package frc.robot;

import java.util.function.Supplier;

import frc.robot.Constants.ESystemState;
import frc.robot.Constants.ArmConstants.EArmState;
import frc.robot.Constants.IntakeConstants.EIntakeState;
import frc.robot.Constants.LEDConstants.ELEDState;
import frc.robot.Constants.ShooterConstants.EAcceleratorState;
import frc.robot.Constants.ShooterConstants.EShooterPivotState;

public class SystemState {
    private ESystemState mSystemState;
    private Supplier<EIntakeState> mIntakeState;
    private Supplier<EShooterPivotState> mShooterPivotState;
    private Supplier<EAcceleratorState> mAcceleratorState;
    private Supplier<EArmState> mArmState;
    private Supplier<ELEDState> mLEDState;

    public SystemState() {
        mSystemState = ESystemState.kDefault;
        mIntakeState = () -> EIntakeState.kStop;
        mShooterPivotState = () -> EShooterPivotState.kCloseShot;
        mAcceleratorState = () -> EAcceleratorState.kStop;
        mArmState = () -> EArmState.kAcceleratorTransfer;
        mLEDState = () -> ELEDState.kVibing;
    }

    public void setSystemState(ESystemState pRobotState) {
        mSystemState = pRobotState;
    }

    public void setIntakeSupplier(Supplier<EIntakeState> pSupplier) {
        mIntakeState = pSupplier;
    }

    public void setShooterPivotSupplier(Supplier<EShooterPivotState> pSupplier) {
        mShooterPivotState = pSupplier;
    }

    public void setAcceleratorSupplier(Supplier<EAcceleratorState> pSupplier) {
        mAcceleratorState = pSupplier;
    }

    public void setArmSupplier(Supplier<EArmState> pSupplier) {
        mArmState = pSupplier;
    }

    public void setLEDSupplier(Supplier<ELEDState> pSupplier) {
        mLEDState = pSupplier;
    }


    public ESystemState getRobotState() {
        return mSystemState;
    }

    public EIntakeState getIntakeState() {
        return mIntakeState.get();
    }

    public EShooterPivotState getShooterPivorState() {
        return mShooterPivotState.get();
    }

    public EAcceleratorState getAcceleratorState() {
        return mAcceleratorState.get();
    }

    public EArmState getArmState() {
        return mArmState.get();
    }

    public ELEDState getLEDState() {
        return mLEDState.get();
    }
}
