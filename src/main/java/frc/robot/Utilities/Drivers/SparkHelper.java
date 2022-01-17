package frc.robot.Utilities.Drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.TrajectoryFollowingMotion.Util;

public class SparkHelper {

    public static boolean setPIDGains(CANSparkMax spark, int slotID, double kP, double kI, double kD, double kF) {
        return setPIDGains(spark, slotID, kP, kI, kD, kF, Constants.kTimeoutMs);
    }

    public static boolean setPIDGains(CANSparkMax spark, int slotID, double kP, double kI, double kD, double kF, int timeout) {
        boolean setSucceeded = true;
        int retryCounter = 0;

        if(timeout > 0) {
            do {
                setSucceeded = true;

                setSucceeded &= spark.getPIDController().setP(kP, slotID) == REVLibError.kOk;
                setSucceeded &= spark.getPIDController().setI(kI, slotID) == REVLibError.kOk;
                setSucceeded &= spark.getPIDController().setD(kD, slotID) == REVLibError.kOk;
                setSucceeded &= spark.getPIDController().setFF(kF, slotID) == REVLibError.kOk;
            } while(!setSucceeded && retryCounter++ < Constants.kSparkMaxRetryCount);
        } else {
            spark.getPIDController().setP(kP, slotID);
            spark.getPIDController().setI(kI, slotID);
            spark.getPIDController().setD(kD, slotID);
            spark.getPIDController().setFF(kF, slotID);
        }

        return retryCounter < Constants.kSparkMaxRetryCount && setSucceeded;
    }

    public static boolean setPIDGains(CANSparkMax spark, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone) {
		return setPIDGains(spark, slotID, kP, kI, kD, kF, rampRate, iZone, Constants.kTimeoutMs);
	}
    public static boolean setPIDGains(CANSparkMax spark, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone, int timeout) {
		boolean setSucceeded = setPIDGains(spark, slotID, kP, kI, kD, kF, timeout);
		int retryCounter = 0;

		if (timeout > 0) {
			do {
				setSucceeded = true;

                setSucceeded &= spark.setClosedLoopRampRate(rampRate) == REVLibError.kOk;
                setSucceeded &= spark.getPIDController().setIZone(iZone, slotID) == REVLibError.kOk;

			} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		} else {
			spark.setClosedLoopRampRate(rampRate);
			spark.getPIDController().setIZone(iZone, slotID);
		}
		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

    public static boolean setPIDGains(SparkMaxU spark, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone) {
		return setPIDGains(spark, slotID, kP, kI, kD, kF, rampRate, iZone, Constants.kTimeoutMs);
	}
    public static boolean setPIDGains(SparkMaxU spark, int slotID, double kP, double kI, double kD, double kF, double rampRate, int iZone, int timeout) {
		boolean setSucceeded = setPIDGains(spark, slotID, kP, kI, kD, kF, timeout);
		int retryCounter = 0;

		if (timeout > 0) {
			do {
				setSucceeded = true;

                setSucceeded &= spark.setClosedLoopRampRate(rampRate) == REVLibError.kOk;
                setSucceeded &= spark.getPIDController().setIZone(iZone, slotID) == REVLibError.kOk;

			} while (!setSucceeded && retryCounter++ < Constants.kTalonRetryCount);
		} else {
			spark.setClosedLoopRampRate(rampRate);
			spark.getPIDController().setIZone(iZone, slotID);
		}
		return retryCounter < Constants.kTalonRetryCount && setSucceeded;
	}

    public static boolean setSmartMotionParams(CANSparkMax spark, int slotID, int cruiseVelocityRPM, int maxAccelRPM) {
        return setSmartMotionParams(spark, slotID, cruiseVelocityRPM, maxAccelRPM);
    }
    public static boolean setSmartMotionParams(CANSparkMax spark, int slotID, int cruiseVelocityRPM, int maxAccelRPM, int timeout) {
        boolean setSucceeded = true;
        int retryCounter = 0;

        if(timeout > 0) {
            do {
                setSucceeded = true;

                setSucceeded &= spark.getPIDController().setSmartMotionMaxVelocity(Util.convertRPMToNativeUnits(cruiseVelocityRPM), slotID) == REVLibError.kOk;
                setSucceeded &= spark.getPIDController().setSmartMotionMaxAccel(Util.convertRPMToNativeUnits(maxAccelRPM), slotID) == REVLibError.kOk;
            } while(!setSucceeded && retryCounter++ < Constants.kSparkMaxRetryCount);
        } else {
            spark.getPIDController().setSmartMotionMaxVelocity(Util.convertRPMToNativeUnits(cruiseVelocityRPM), slotID);
            spark.getPIDController().setSmartMotionMaxAccel(Util.convertRPMToNativeUnits(maxAccelRPM), slotID);
        }

        return retryCounter < Constants.kSparkMaxRetryCount && setSucceeded;
    }

    public static boolean setSmartMotionParams(SparkMaxU spark, int slotID, int cruiseVelocityRPM, int maxAccelRPM) {
        return setSmartMotionParams(spark, slotID, cruiseVelocityRPM, maxAccelRPM);
    }
    public static boolean setSmartMotionParams(SparkMaxU spark, int slotID, int cruiseVelocityRPM, int maxAccelRPM, int timeout) {
        boolean setSucceeded = true;
        int retryCounter = 0;

        if(timeout > 0) {
            do {
                setSucceeded = true;

                setSucceeded &= spark.getPIDController().setSmartMotionMaxVelocity(Util.convertRPMToNativeUnits(cruiseVelocityRPM), slotID) == REVLibError.kOk;
                setSucceeded &= spark.getPIDController().setSmartMotionMaxAccel(Util.convertRPMToNativeUnits(maxAccelRPM), slotID) == REVLibError.kOk;
            } while(!setSucceeded && retryCounter++ < Constants.kSparkMaxRetryCount);
        } else {
            spark.getPIDController().setSmartMotionMaxVelocity(Util.convertRPMToNativeUnits(cruiseVelocityRPM), slotID);
            spark.getPIDController().setSmartMotionMaxAccel(Util.convertRPMToNativeUnits(maxAccelRPM), slotID);
        }

        return retryCounter < Constants.kSparkMaxRetryCount && setSucceeded;
    }
    
}
