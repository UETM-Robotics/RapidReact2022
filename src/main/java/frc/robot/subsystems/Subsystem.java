package frc.robot.subsystems;

import frc.robot.Utilities.Loops.Looper;

public abstract class Subsystem {
    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
    }

    public abstract void stop();

    public void zeroSensors() {
    }

    public void registerEnabledLoops(Looper enabledLooper) {
    }
}