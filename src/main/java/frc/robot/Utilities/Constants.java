package frc.robot.Utilities;

public class Constants {
    // Wheels
	public static final double kDriveWheelDiameterInches = 6.0;	//Practice bot calibrated 4.875
	//public static final double kDriveWheelDiameterInches = 5;	//Comp bot measured val
	//public static final double kDriveWheelDiameterInches = PathAdapter.getAdaptedWheelDiameter();
	public static final double kTrackWidthInches = 25.25;
	public static final double kTrackScrubFactor = 1.0; // 0.924 ?

	// Geometry
	public static final double kCenterToFrontBumperDistance = 18.75;
	public static final double kCenterToIntakeDistance = 18.75;
	public static final double kCenterToRearBumperDistance = 18.75;
	public static final double kCenterToSideBumperDistance = 16.375;

    // Path following constants
	public static final double kMinLookAhead = 12.0; // inches
	public static final double kMinLookAheadSpeed = 9.0; // inches per second
	public static final double kMaxLookAhead = 24.0; // inches
	public static final double kMaxLookAheadSpeed = 140.0; // inches per second
	public static final double kDeltaLookAhead = kMaxLookAhead - kMinLookAhead;
	public static final double kDeltaLookAheadSpeed = kMaxLookAheadSpeed - kMinLookAheadSpeed;

	public static final double kInertiaSteeringGain = 0.0; // angular velocity command is multiplied by this gain *
	// our speed
	// in inches per sec
	public static final double kSegmentCompletionTolerance = 1; // inches
	public static final double kPathFollowingMaxAccel = 100.0; // inches per second^2
	public static final double kPathFollowingMaxVel = 140.0; // inches per second

	public static final double kPathFollowingProfileKp = 5.0;   //Used to be 5 when tuning our paths
	public static final double kPathFollowingProfileKi = 0.03;
	public static final double kPathFollowingProfileKv = 0.2;
	public static final double kPathFollowingProfileKffv = 1.0;
	public static final double kPathFollowingProfileKffa = 0.05;
	public static final double kPathFollowingGoalPosTolerance = 1;
	public static final double kPathFollowingGoalVelTolerance = 18.0;
	public static final double kPathStopSteeringDistance = 9.0;

	// Goal tracker constants
	public static final double kMaxGoalTrackAge = 1.0;
	public static final double kMaxTrackerDistance = 18.0;
	public static final double kCameraFrameRate = 30.0;
	public static final double kTrackReportComparatorStablityWeight = 1.0;
	public static final double kTrackReportComparatorAgeWeight = 1.0;

	// Pose of the camera frame w.r.t. the robot frame
	public static final double kCameraXOffset = -3.3211;
	public static final double kCameraYOffset = 0.0;
	public static final double kCameraZOffset = 20.9;
	public static final double kCameraPitchAngleDegrees = 29.56; // Measured on 4/26
	public static final double kCameraYawAngleDegrees = 0.0;
	public static final double kCameraDeadband = 0.0;
    public static final double kCameraLensHeightToTargetHeightDelta = 0; //TODO: CALC HEIGHT DIFFERENCE

	// Target parameters
	// Source of current values: https://firstfrc.blob.core.windows.net/frc2017/Manual/2017FRCGameSeasonManual.pdf
	// Section 3.13
	// ...and https://firstfrc.blob.core.windows.net/frc2017/Drawings/2017FieldComponents.pdf
	// Parts GE-17203-FLAT and GE-17371 (sheet 7)
	public static final double kBoilerTargetTopHeight = 88.0;
	public static final double kBoilerRadius = 7.5;


	public static final int kLooperThreadPriority = Thread.MAX_PRIORITY;

	public static final int kTimeoutMs = 20;
	public static final int kActionTimeoutS = 2;
	public static final int kTalonRetryCount = 3;
	public static final double kJoystickDeadband = 0.08;
	public static final double kWheelDeadband = 0.05;
	public static final double kSensorUnitsPerRotation = 4096.0;
	public static final double k100msPerMinute = 600.0;
	public static final double kLooperDt = 0.005;

	public static final int kTimeoutMsFast = 10;
	public static int kSparkMaxRetryCount = 3;

	public static final double kDriveTrainMaxVelocityMPS = 4.2;
	public static final double kDriveTrainMaxVelocityRPM = 5637;
	

	/* CONTROL LOOP GAINS */

	// PID gains for drive velocity loop (HIGH GEAR)
	// Units: setpoint, error, and output are in inches per second.
	public static final double kDriveHighGearVelocityKp = 0.2;
	public static final double kDriveHighGearVelocityKi = 0;//0.005;
	public static final double kDriveHighGearVelocityKd = 0.3;
	public static final double kDriveHighGearVelocityKf = 0.165;
	public static final int kDriveHighGearVelocityIZone = 0;
	public static final double kDriveHighGearVelocityRampRate = 0.1;
	public static final double kDriveHighGearMaxSetpoint = 12.0 * 12.0; // 12 fps

	//PID gains for shooter velocity loop
	//Units: setpoint, error, and output are in rpm
	public static final double kShooterVelocityKp = 0.00222;
	public static final double kShooterVelocityKi = 0.0;
	public static final double kShooterVelocityKd = 0.00444;
	public static final double kShooterVelocityKf = 0.00026;
	public static final double kShooterVelocityRampRate = 0.2;
	public static final int kShooterMaxAccel = 5200;
	public static final double kBeltTransporterVelocity = 0.75;

	//PID gains for intake velocity loop
	//Units: setpoint, error, and output are in rpm
	public static final double kBeltVelocityKp = 0.0;
	public static final double kBeltVelocityKi = 0.0;
	public static final double kBeltVelocityKd = 0.0;
	public static final double kBeltVelocityKf = 0.0;
	public static final double kBeltVelocityRampRate = 0.2;
	public static final int kBeltMaxAccel = 0;
}