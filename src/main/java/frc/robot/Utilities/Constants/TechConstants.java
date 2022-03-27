package frc.robot.Utilities.Constants;

import com.revrobotics.SparkMaxPIDController.AccelStrategy;

public class TechConstants {
    
    public static final int kControllerThreadPriority = 9;

    public static double kControllerTriggerThreshold = 0.75;
    public static final double kJoystickDeadband = 0.1;

    public static final double kJoystickJogThreshold = 0.4;
    public static final int kSparkMaxRetryCount = 3;
    public static final int kTimeoutMs = 20;
    public static final int kTimeoutMsFast = 10;

    public static final double kLooperDt = 0.005;
    public static final int kLooperThreadPriority = 10;

    public static final double kSensorUnitsPerRotation = 1.0;
    public static final double k100msPerMinute = 1.0;


    public static final double kImageCaptureLatency = 11.0 / 1000.0;
    public static final double kMaxTrackerDistance = 18.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kCameraFrameRate = 90.0;

    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));

    //TODO: FIGURE THIS NUMBER OUT
    public static final double kHubTargetHeight = 104.0; //inches
    public static final double kHubPositionXMeters = 8.24;
    public static final double kHubPositionYMeters = 4.1;


    public static final double kPathFollowingMaxAccel = 80.0;  // inches per second ^ 2
    public static final double kPathFollowingMaxVel = 120.0; // inches per second
    public static final double kPathFollowingProfileKp = 0.3 / 12.0;  // % throttle per inch of error
    public static final double kPathFollowingProfileKi = 0.0;
    public static final double kPathFollowingProfileKv = 0.01 / 12.0;  // % throttle per inch/s of error
    public static final double kPathFollowingProfileKffv = 0.003889;  // % throttle per inch/s
    public static final double kPathFollowingProfileKffa = 0.001415;  // % throttle per inch/s^2
    public static final double kPathFollowingProfileKs = 0.1801 / 12.0;  // % throttle
    public static final double kPathFollowingGoalPosTolerance = 3.0;
    public static final double kPathFollowingGoalVelTolerance = 12.0;
    public static final double kPathStopSteeringDistance = 12.0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static final int kDriveCurrentThrottledLimit = 30; // amps
    public static final int kDriveCurrentUnThrottledLimit = 80; // amps

    public static final double kDriveVelocityKp = 0.0015;
	public static final double kDriveVelocityKi = 0;//0.005;
	public static final double kDriveVelocityKd = 0;
	public static final double kDriveVelocityKf = 0.005;
	public static final int kDriveVelocityIZone = 0;
	public static final double kDriveVelocityRampRate = 0.1;
    public static final AccelStrategy kDriveAccelStrategy = AccelStrategy.kTrapezoidal;
    public static final double kDriveMaxVelocity = 4.0;
    public static final double kDriveMaxAccel = 2.0;

    public static final double kShooterVelocityKp = 0;
    public static final double kShooterVelocityKi = 0;
    public static final double kShooterVelocityKd = 0;
    public static final double kShooterVelocityKf = 0;
    public static final int kShooterVelocityIZone = 0;
    public static final double kShooterVelocityClosedLoopRampRate = 0;
    public static final AccelStrategy kShooterAccelStrategy = AccelStrategy.kTrapezoidal;
    public static final double kShooterMaxVelocity = 0;
    public static final double kShooterMaxAccel = 0;

    public static final double kShooterArmVelocityPercentage = 0.98;
    public static final double kShooterTriggerVelocityPercentage = 0.95;


    public static final double kHoodVelocityKp = 0;
    public static final double kHoodVelocityKi = 0;
    public static final double kHoodVelocityKd = 0;
    public static final double kHoodVelocityKf = 0;
    public static final int kHoodVelocityIZone = 0;
    public static final double kHoodVelocityClosedLoopRampRate = 0.2;
    public static final AccelStrategy kHoodAccelStrategy = AccelStrategy.kTrapezoidal;
    public static final double kHoodMaxVelocity = 0;
    public static final double kHoodMaxAccel = 0;
    public static final double kHoodSmartMotionAllowedClosedLoopError = 1.0;

    public static final double kDriveRotationalKp = 0.015;
    public static final double kDriveRotationalKi = 0.0;
    public static final double kDriveRotationalKd = 0.0;
    public static final double llTargetingOffset = -0.07;
    public static final double llTargetingDeadband = 0.05;
    
}
