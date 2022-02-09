package frc.robot.subsystems;


import frc.robot.Utilities.Geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Utilities.Constants;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.Geometry.Translation2d;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.TrajectoryFollowingMotion.Rotation2d;

public class Vision extends Subsystem implements CustomSubsystem{


    private static Vision instance = new Vision();
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private TargetMode mTargetMode = TargetMode.RED_CARGO;
    private boolean visionEnabled = false;

    private double[] zeroArray = new double[] {0, 0, 0, 0};

    private NetworkTable currentLimelightNT;

    private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry pipelineEntry = limelight.getEntry("pipeline");

    
    private final ElapsedTimer loopTimer = new ElapsedTimer();


    public static Vision getInstance() {
        return instance;
    }


    private Vision() {}


    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            synchronized (Vision.this) {

            }
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (Vision.this) {
                
            }
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {

            loopTimer.start();
            
            synchronized (Vision.this) {

                switch(mTargetMode) {

                    case BLUE_CARGO:
                        mPeriodicIO.pipeline_front = visionEnabled ? 1 : 0;
                        currentLimelightNT = limelight;
                        break;
                    case HUB:
                        mPeriodicIO.pipeline_front = visionEnabled ? 2 : 0;
                        currentLimelightNT = limelight;
                        break;
                    case RED_CARGO:
                        mPeriodicIO.pipeline_front = visionEnabled ? 3 : 0;
                        currentLimelightNT = limelight;
                        break;
                    case CAM:
                        mPeriodicIO.pipeline_front = 0;
                        currentLimelightNT = limelight;
                    default:
                        mPeriodicIO.pipeline_front = 0;
                        break;

                }

                readPeriodicInputs();
                writePeriodicOutputs();

                mPeriodicIO.vision_loop_time += loopTimer.hasElapsed();
            }   
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
        
    };


    @Override
    public void init() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void subsystemHome() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
        
    }


    public void stop() {
        setVisionEnabled(false);
    }

    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    public boolean isTargetFound() {
        return visionEnabled && mPeriodicIO.target_valid > 0;
    }


    //TODO: IMPLEMENT TARGET DISTANCE ESTIMATION
    public double getTargetDistance() {
        return visionEnabled ? mPeriodicIO.target_distance : 0;
    }

    public double getTargetHorizAngleDev() {
		return visionEnabled ? mPeriodicIO.target_horizontal_deviation : 0;
	}

	public double getTargetVertAngleDev() {
		return visionEnabled ? mPeriodicIO.target_vertical_deviation : 0;
	}

    //TODO: IMPLEMENT TARGET DISTANCE ESTIMATION

    public synchronized void setVisionEnabled(boolean enabled) {
		visionEnabled = enabled;
	}

	public synchronized void setTargetMode(TargetMode targetMode) {
		mTargetMode = targetMode;
	}

    public double getTargetSkew() {
		return mPeriodicIO.target_skew;
	}

	public Pose2d getCameraToTargetPose() {
		return mPeriodicIO.camera_to_target_pose;
	}


    public Translation2d getCameraToTargetTranslation() {
		// Convert to spherical coordinates https://en.wikipedia.org/wiki/Spherical_coordinate_system
		Rotation2d phi = Rotation2d.fromDegrees( -1 * mPeriodicIO.target_horizontal_deviation );
        Rotation2d theta = Rotation2d.fromDegrees(90).rotateBy(Rotation2d.fromDegrees( mPeriodicIO.target_vertical_deviation + Constants.kCameraPitchAngleDegrees).inverse());

		// Convert to cartesian unit vector (radius r is implicitly 1, inclination theta, azimuth phi)
		double vector_x = theta.sin() * phi.cos();
		double vector_y = theta.sin() * phi.sin();
		double vector_z = theta.cos();

		// Determine the scaling of the z component of the unit vector to get to the plane
		double scaling = Constants.kCameraLensHeightToTargetHeightDelta / vector_z;

		// Scale the x and y component by said scaling.
		return new Translation2d(vector_x * scaling, vector_y * scaling);
	}


    public void readPeriodicInputs() {
        loopTimer.start();
        try {

            if(visionEnabled) {

                mPeriodicIO.target_valid = currentLimelightNT.getEntry("tv").getDouble(0);
                mPeriodicIO.target_horizontal_deviation = currentLimelightNT.getEntry("tx").getDouble(0);
				mPeriodicIO.target_vertical_deviation = currentLimelightNT.getEntry("ty").getDouble(0);
				mPeriodicIO.target_area = currentLimelightNT.getEntry("ta").getDouble(0);
				mPeriodicIO.target_skew = currentLimelightNT.getEntry("ts").getDouble(0);
				mPeriodicIO.target_latency = currentLimelightNT.getEntry("tl").getDouble(0);
				mPeriodicIO.x_corners = limelight.getEntry("tcornx").getDoubleArray(zeroArray);
				mPeriodicIO.y_corners = limelight.getEntry("tcorny").getDoubleArray(zeroArray);

            } else {

				mPeriodicIO.target_valid = 0;
				mPeriodicIO.target_horizontal_deviation = 0;
				mPeriodicIO.target_vertical_deviation = 0;
				mPeriodicIO.target_area = 0;
				mPeriodicIO.target_skew = 0;
				mPeriodicIO.target_latency = 0;
				mPeriodicIO.camera_to_target_pose = Pose2d.identity();
				mPeriodicIO.target_distance = 0;

			}

            

        } catch(Exception ex) {
            System.out.println("Fatal Error Reading Limelight Inputs");
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

    @Override
	public synchronized void writePeriodicOutputs() {
		loopTimer.start();
		try {
			pipelineEntry.setNumber(mPeriodicIO.pipeline_front);
		}
		catch (Exception ex) {
            System.out.println("-------------------------------------------");
            System.out.println("Fatal Error writing Vision Periodic Outputs");
            System.out.println("-------------------------------------------");
		}

		mPeriodicIO.vision_loop_time += loopTimer.hasElapsed();
	}



    public enum TargetMode {
        RED_CARGO,
        BLUE_CARGO,
        HUB,
        CAM
    }
    
}
