package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Geometry.Translation2d;

public class Vision extends Subsystem implements CustomSubsystem {

    

    private static Vision instance = new Vision();
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private TargetMode mTargetMode = TargetMode.HUB;
	private boolean mVisionEnabled = true;

    private double[] mZeroArray = new double[]{0, 0, 0, 0};

    private NetworkTable mCurrentTargetingLimelightNT = NetworkTableInstance.getDefault().getTable("limelight");

	private NetworkTable limelightShooter = NetworkTableInstance.getDefault().getTable("limelight");
	//private NetworkTableEntry pipelineEntry = limelightShooter.getEntry("pipeline");

	//TODO: REMEMBER TO SET THESE BEFORE ATTEMPTING ANYTHING!!!
	private final int offsetPipeline = 0;
	private final int nonOffsetPipeline = 0;

    private final ElapsedTimer loopTimer = new ElapsedTimer();

	public boolean offsetEnabled;


    public static Vision getInstance() {
        return instance;
    }


    private Vision() {
		offsetEnabled = true;
    }


    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            synchronized(Vision.this) {

            }
        }

        @Override
        public void onStart(double timestamp) {
            synchronized(Vision.this) {

            }
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            loopTimer.start();
            synchronized(Vision.this) {

				switch(mTargetMode) {
					case BlUE_CARGO:
						//TODO: FOR POSTERITY PURPOSES
						break;
					case HUB:
						mPeriodicIO.pipeline_front = mVisionEnabled ? 0 : 0;
						break;
					case RED_CARGO:
						break;
					default:
						break;

				}

            }

			

			readPeriodicInputs();
            mPeriodicIO.vision_loop_time += loopTimer.hasElapsed();
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }

    };

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }


    @Override
    public void stop() {
        setVisionEnabled(false);
    }

    public boolean isVisionEnabled() {
		return mVisionEnabled;
	}

	public boolean isTargetFound() {
		return mVisionEnabled && mPeriodicIO.target_valid > 0;
	}

	public double getTargetDistance() {
		return mVisionEnabled ? mPeriodicIO.target_distance : 0;
	}

	public double getTargetHorizAngleDev() {
		return mVisionEnabled ? mPeriodicIO.target_horizontal_deviation : 0;
	}

	public double getTargetVertAngleDev() {
		return mVisionEnabled ? mPeriodicIO.target_vertical_deviation : 0;
	}

	public synchronized void setVisionEnabled(boolean enabled) {
		mVisionEnabled = enabled;
	}

	public synchronized void setTargetMode(TargetMode targetMode) {
		mTargetMode = targetMode;
	}

	public synchronized void setLEDS(boolean enabled) {
		mCurrentTargetingLimelightNT.getEntry("ledMode").setNumber(enabled ? 3 : 1);
	}

	public double getTargetSkew() {
		return mPeriodicIO.target_skew;
	}

    public Translation2d getCameraToTargetTranslation() {
		// Convert to spherical coordinates https://en.wikipedia.org/wiki/Spherical_coordinate_system
		Rotation2d phi = Rotation2d.fromDegrees(-1 * mPeriodicIO.target_horizontal_deviation);
		Rotation2d theta = Rotation2d.fromDegrees(90).rotateBy(Rotation2d.fromDegrees(mPeriodicIO.target_vertical_deviation + TechConstants.kCameraLensAngleToHorizontal).inverse());

		// Convert to cartesian unit vector (radius r is implicitly 1, inclination theta, azimuth phi)
		double vector_x = theta.sin() * phi.cos();
		double vector_y = theta.sin() * phi.sin();
		double vector_z = theta.cos();

		// Determine the scaling of the z component of the unit vector to get to the plane
		double scaling = TechConstants.kCameraLensHeightToTargetHeightDelta / vector_z;

		// Scale the x and y component by said scaling.
		return new Translation2d(vector_x * scaling, vector_y * scaling);
	}


	public synchronized void setOffsetEnabled(boolean enabled) {
		if(enabled == offsetEnabled)
			return;
		
		mCurrentTargetingLimelightNT.getEntry("pipeline").setNumber(enabled ? offsetPipeline : nonOffsetPipeline);
	}

	public synchronized boolean getOffsetEnabled() {
		return offsetEnabled;
	}


	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		try {
			if (mVisionEnabled) {
				mPeriodicIO.target_valid = mCurrentTargetingLimelightNT.getEntry("tv").getDouble(0);
				mPeriodicIO.target_horizontal_deviation = mCurrentTargetingLimelightNT.getEntry("tx").getDouble(0);
				mPeriodicIO.target_vertical_deviation = mCurrentTargetingLimelightNT.getEntry("ty").getDouble(0);
				mPeriodicIO.target_area = mCurrentTargetingLimelightNT.getEntry("ta").getDouble(0);
				mPeriodicIO.target_skew = mCurrentTargetingLimelightNT.getEntry("ts").getDouble(0);
				mPeriodicIO.target_latency = mCurrentTargetingLimelightNT.getEntry("tl").getDouble(0);
				mPeriodicIO.x_corners = limelightShooter.getEntry("tcornx").getDoubleArray(mZeroArray);
				mPeriodicIO.y_corners = limelightShooter.getEntry("tcorny").getDoubleArray(mZeroArray);

				if (mPeriodicIO.target_valid > 0) {
					Translation2d cameraToTargetTranslation = getCameraToTargetTranslation();
                    //TODO: FIX TURRET REFERENCES
					// mPeriodicIO.camera_to_target_pose = Constants.fieldToOuterTarget.transformBy(
					// 		new Pose2d(cameraToTargetTranslation.translateBy(Constants.kTurretToCamera.inverse().getTranslation()), Shooter.getInstance().getLatestFieldToTurretPose().getRotation()).inverse()
					// 				.transformBy(Shooter.getInstance().getLatestVehicleToTurretPose().inverse())
					// );
					//RobotState.getInstance().addFieldToVehicleObservation(Timer.getFPGATimestamp(), mPeriodicIO.camera_to_target_pose);
					mPeriodicIO.target_distance = cameraToTargetTranslation.distance(Translation2d.identity());
				}
			}
			else {
				mPeriodicIO.target_valid = 0;
				mPeriodicIO.target_horizontal_deviation = 0;
				mPeriodicIO.target_vertical_deviation = 0;
				mPeriodicIO.target_area = 0;
				mPeriodicIO.target_skew = 0;
				mPeriodicIO.target_latency = 0;
				mPeriodicIO.camera_to_target_pose = Pose2d.identity();
				mPeriodicIO.target_distance = 0;
			}
		}
		catch (Exception ex) {
            DriverStation.reportError("Error setting vision outputs", true);
		}
		mPeriodicIO.vision_loop_time = loopTimer.hasElapsed();
	}


    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		//Read values
		public double target_valid;
		public double target_horizontal_deviation;
		double target_vertical_deviation;
		public double target_area;
		double target_skew;
		double target_latency;
		double target_distance;
		double[] x_corners;
		double[] y_corners;
		public Pose2d camera_to_target_pose;

		//Written values
		int pipeline_front;
		public double vision_loop_time;
	}

	public enum TargetMode {
		HUB,
        RED_CARGO,
        BlUE_CARGO;
	}

    @Override
    public void init() {
        // TODO Auto-generated method stub
        
    }


    @Override
    public void subsystemHome() {
        // TODO Auto-generated method stub
        
    }

    
}
