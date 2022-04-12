package frc.robot.Utilities.Constants;

import com.revrobotics.SparkMaxPIDController.AccelStrategy;

public class TechConstants {

    public static final double kLooperDt = 0.005;
    public static final int kLooperThreadPriority = 10;
    public static final int kControllerThreadPriority = 9;
    
    public static final double kJoystickJogThreshold = 0.4;
    public static final double kControllerTriggerThreshold = 0.75;
    public static final int kSparkMaxRetryCount = 3;
    public static final int kTimeoutMs = 20;
    public static final int kTimeoutMsFast = 10;

    public static final double kHubPositionXMeters = 8.7;
    public static final double kHubPositionYMeters = 4.1;
    public static final double kJoystickDeadband = 0.1;

    public static final double kRotationsToMetersConversionFactor = (0.1524 * Math.PI) / 10.71;
    public static final double kRPMtoMetersPerSecondConversionFactor = (0.1524 * Math.PI) / 60.0 / 10.71;

    public static final double kDriveVelocityKp = 0.0015;
	public static final double kDriveVelocityKi = 0;//0.005;
	public static final double kDriveVelocityKd = 0;
	public static final double kDriveVelocityKf = 0.005;
	public static final int kDriveVelocityIZone = 0;
	public static final double kDriveVelocityRampRate = 0.1;
    public static final AccelStrategy kDriveAccelStrategy = AccelStrategy.kTrapezoidal;
    public static final double kDriveMaxVelocity = 4.0;
    public static final double kDriveMaxAccel = 2.0;
    
}
