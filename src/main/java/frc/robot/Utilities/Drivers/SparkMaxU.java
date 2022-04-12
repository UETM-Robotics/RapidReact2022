package frc.robot.Utilities.Drivers;


import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;

import frc.robot.Utilities.Constants.TechConstants;

public class SparkMaxU extends CANSparkMax{
    private int currentSlectedSlot = 0;

    private double[] mCLRampRate = {0, 0};
    private int[] mSMAccel = {0, 0};
    private int[] mSMVel = {0, 0};

    private SparkMaxPIDController pid;

    public SparkMaxU(int deviceId, int pdpChannel) {
        super(deviceId, MotorType.kBrushless);

        pid = super.getPIDController();
    }

    public void set(double outputValue, ControlType type) {
        pid.setReference(outputValue, type);
    }

    @Override
    public void set(double outputValue) {
        super.set(outputValue);
    }

    public void resetEncoder() {
        super.getEncoder().setPosition(0);
    }

    @Override
    public REVLibError restoreFactoryDefaults() {
        return super.restoreFactoryDefaults();
    }

    public REVLibError configClosedloopRamp(double secondsFromNeutralToFull, int timeoutMs) {
        return configClosedloopRamp(secondsFromNeutralToFull, currentSlectedSlot, timeoutMs);
    }

    public REVLibError configClosedloopRamp(double secondsFromNeutralToFull, int slotIdx, int timeoutMs) {
        setCurrentSlotCLRampRate(secondsFromNeutralToFull, slotIdx);
        return super.setClosedLoopRampRate(secondsFromNeutralToFull);
    }

    public REVLibError configMotionAcceleration(int sensorUnitsPer100msPerSec, int timeoutMs) {
        return configMotionAcceleration(sensorUnitsPer100msPerSec, currentSlectedSlot, timeoutMs);
    }

    public REVLibError configMotionAcceleration(int sensorUnitsPer100msPerSec, int slotIdx, int timeoutMs) {
        setCurrentSMAccel(sensorUnitsPer100msPerSec, slotIdx);
        return pid.setSmartMotionMaxAccel(sensorUnitsPer100msPerSec, 1);
    }

    public REVLibError configMotionCruiseVlocity(int sensorUnitsPer100ms, int timeoutMs) {
        return configMotionCruiseVlocity(sensorUnitsPer100ms, currentSlectedSlot, timeoutMs);
    }

    public REVLibError configMotionCruiseVlocity(int sensorUnitsPer100ms, int slotIdx, int timeoutMs) {
        setCurrentSMVel(sensorUnitsPer100ms, slotIdx);
        return pid.setSmartMotionMaxVelocity(sensorUnitsPer100ms, 1);
    }

    public void selectProfileSlot(int slotIdx, int pidIdx) {
        setCurrentSlotValue(slotIdx);
        if(currentSlectedSlot < mCLRampRate.length && currentSlectedSlot < mSMAccel.length && currentSlectedSlot < mSMVel.length) {
            boolean setSuceeded;
            int retryCounter = 0;

            do {
                setSuceeded = configClosedloopRamp(mCLRampRate[currentSlectedSlot], currentSlectedSlot, TechConstants.kTimeoutMsFast) == REVLibError.kOk;
                setSuceeded &= configMotionAcceleration(mSMAccel[currentSlectedSlot], currentSlectedSlot, TechConstants.kTimeoutMsFast) == REVLibError.kOk;
                setSuceeded &= configMotionCruiseVlocity(mSMVel[currentSlectedSlot], currentSlectedSlot, TechConstants.kTimeoutMsFast) == REVLibError.kOk;
            } while(!setSuceeded && retryCounter++ < TechConstants.kSparkMaxRetryCount);

        }
    }

    public void set(ControlType type, double outputValue, int slotIdx) {
        if(currentSlectedSlot != slotIdx) {
            selectProfileSlot(slotIdx, 0);
        }

        set(outputValue, type);
    }

    private synchronized void setCurrentSlotValue(int slotIdx) {
        currentSlectedSlot = slotIdx;
    }

    private synchronized void setCurrentSlotCLRampRate(double rampRate, int slot) {
		if (slot < mCLRampRate.length) {
			mCLRampRate[slot] = rampRate;
		}
	}

    private synchronized void setCurrentSMVel(int vel, int slot) {
		if (slot < mSMVel.length) {
			mSMVel[slot] = vel;
		}
	}

    private synchronized void setCurrentSMAccel(int accel, int slot) {
		if (slot < mSMAccel.length) {
			mSMAccel[slot] = accel;
		}
	}

}