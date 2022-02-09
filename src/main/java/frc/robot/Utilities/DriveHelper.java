package frc.robot.Utilities;

import frc.robot.Constants.DriverConstants;

public class DriveHelper {

    private static final double kThrottleDeadband = DriverConstants.kJoystickDeadband;
    private static final double kWheelDeadband = DriverConstants.kWheelDeadband;

    private static final double kWheelNonLinearity = 0.65;

    private static final double kNegInertiaScalar = 4.0;

    private static final double kInertiaScalar = 0.65;
    private static final double kInertiaTurnScalar = 3.5;
    private static final double kInertiaCloseScalar = 4.0;
    private static final double kInertiaFarScalar = 5.0;

    private static final double kSensitivity = 0.95;

    private static final double kQuickStopDeadband = 0.2;
    private static final double kQuickStopWeight = 0.1;
    private static final double kQuickStopScalar = 5.0;

    private double mOldWheel = 0.0;
    private double mQuickStopAccumlator = 0.0;
    private double mNegInertiaAccumlator = 0.0;

    public DriveMotorValues calculateOutput(double throttle, double wheel, boolean isQuickTurn) {
        return calculateOutput(throttle, wheel, isQuickTurn, 1.0);
    }

    public DriveMotorValues calculateOutput(double throttle, double wheel, boolean isQuickTurn, double scalingFactor) {
        
        wheel = QuickMaths.normalizeJoystickWithDeadband(wheel, kWheelDeadband);
        throttle = QuickMaths.normalizeJoystickWithDeadband(throttle, kThrottleDeadband);

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonLinearity);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonLinearity * wheel) / denominator;
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonLinearity * wheel) / denominator;

        double leftPWM, rightPWM, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        double negInertiaPower = negInertia * kNegInertiaScalar;
        sensitivity = kSensitivity;
        mNegInertiaAccumlator += negInertiaPower;

        wheel = wheel + mNegInertiaAccumlator;
        if(mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if(mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }

        linearPower = throttle;

        if(isQuickTurn) {
            if(Math.abs(linearPower) < kQuickStopDeadband) {
                double alpha = kQuickStopWeight;
                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator + alpha * limit(wheel, 1.0) * kQuickStopScalar;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        
        }

        rightPWM = leftPWM = linearPower;
        leftPWM += angularPower;
        rightPWM -= angularPower;

        if (leftPWM > 1.0) {
            rightPWM -= overPower * (leftPWM - 1.0);
            leftPWM = 1.0;
        } else if (rightPWM > 1.0) {
            leftPWM -= overPower * (rightPWM - 1.0);
            rightPWM = 1.0;
        } else if (leftPWM < -1.0) {
            rightPWM += overPower * (-1.0 - leftPWM);
            leftPWM = -1.0;
        } else if (rightPWM < -1.0) {
            leftPWM += overPower * (-1.0 - rightPWM);
            rightPWM = -1.0;
        }
        
        return new DriveMotorValues(leftPWM * scalingFactor, rightPWM * scalingFactor);

    }

    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }
    
}
