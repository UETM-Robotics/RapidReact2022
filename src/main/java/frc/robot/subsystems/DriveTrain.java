package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.DriveSignal;
import frc.robot.Utilities.SynchronousPIDF;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.SparkHelper;
//import frc.robot.Utilities.Drivers.NavX;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.RamseteTrajectory.DifferentialDriveWheelSpeeds;
import frc.robot.Utilities.RamseteTrajectory.RamseteController;
import frc.robot.Utilities.RamseteTrajectory.Trajectory;


public class DriveTrain extends Subsystem implements CustomSubsystem {


    private static final DriveTrain instance = new DriveTrain();
    private final Controllers controllers = Controllers.getInstance();


    public static DriveTrain getInstance() {
        return instance;
    }

    private DriveControlState mControlMode = DriveControlState.DISABLED;
    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private final SparkMaxU leftFront, rightFront;
    private final SparkMaxU leftHind, rightHind;

    //private NavX mNavXBoard;
    private AHRS mNavXBoard;

    private ControllerU driverController;


    private Trajectory mCurrentTrajectory;
    private RamseteController mController;

    private SynchronousPIDF rotationalPIDF;
    private double mHubTargetingAngleGuess = 0;


    private double mThrottle = 0;
    private double mTurn = 0;
    private boolean mPrevBrakeModeVal = false;

    private final PeriodicIO mPeriodicIO = new PeriodicIO();


    private DriveTrain() {

        leftFront = controllers.getLeftFrontDriveMotor();
        rightFront = controllers.getRightFrontDriveMotor();

        leftHind = controllers.getLeftHindDriveMotor();
        rightHind = controllers.getRightHindDriveMotor();

        mNavXBoard = controllers.getNavX();


        rotationalPIDF = new SynchronousPIDF();

        //m_odometry = new DifferentialDriveOdometry(getRotation(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        driverController = controllers.getDriverController();
    }


    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onStart(double timestamp) {
            subsystemHome();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized(DriveTrain.this) {

                SmartDashboard.putNumber("Robot Position X", RobotState.getInstance().getFieldToVehicleMeters().x());
                SmartDashboard.putNumber("Robot Position Y", RobotState.getInstance().getFieldToVehicleMeters().y());
                SmartDashboard.putNumber("Robot Heading", RobotState.getInstance().getFieldToVehicleMeters().getRotation().getDegrees());

                //m_robotPose = m_odometry.update(getRotation(), getLeftDistanceMeters(), getRightDistanceMeters());
               
                switch(mControlMode) {

                    case AUTO_AIMING:

                        updateAutoAiming();

                        break;
                    case DRIVER_CONTROL:

                        mThrottle = -driverController.getNormalizedAxis(1, TechConstants.kJoystickDeadband);
                        mTurn = driverController.getNormalizedAxis(2, TechConstants.kJoystickDeadband) * 0.4;
                        //dTrain.setBrakeMode(driverController.getRawButton(5));
                        //dTrain.setBrakeMode(driverController.getTriggerPressed(2, Constants.kControllerTriggerThreshold));
                        //DriveSignalOutput.set(Math.max(Math.min(mThrottle + mTurn, 1), -1), Math.max(Math.min(mThrottle - mTurn, 1), -1));
                        leftFront.set(Math.max(Math.min(mThrottle + mTurn, 1), -1));
                        rightFront.set( Math.max(Math.min(mThrottle - mTurn, 1), -1));

                        break;
                    case OPEN_LOOP:

                        leftFront.set(mPeriodicIO.left_driveOpenLoopDemand);
                        rightFront.set(mPeriodicIO.right_driveOpenLoopDemand);

                        break;
                    case PATH_FOLLOWING:
                        //updateTrajectoryFollower(timestamp);
                        break;
                    case DISABLED:
                        break;
                    default:
                        directlySetDrivePower(DriveSignal.NEUTRAL);
                        DriverStation.reportError("Anomaly in setting DriveTrain Control State", true);
                        break;

                }
                
            }
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }

    };

    private synchronized void directlySetDrivePower(DriveSignal driveSignal) {
        leftFront.set(driveSignal.getLeft());
        rightFront.set(driveSignal.getRight());
    }

    public synchronized void setBrakeMode(boolean brakeMode) {
		if (mPrevBrakeModeVal != brakeMode) {
			_subsystemMutex.lock();

            leftFront.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
            leftHind.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

            rightFront.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
            rightHind.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
			mPrevBrakeModeVal = brakeMode;
			_subsystemMutex.unlock();
		}
	}

	public synchronized void setDriveOpenLoop(DriveSignal d) {

		if(mControlMode != DriveControlState.OPEN_LOOP) {
			setBrakeMode(false);
			setControlMode(DriveControlState.OPEN_LOOP);
		}

		mPeriodicIO.left_driveOpenLoopDemand = d.getLeft();
		mPeriodicIO.right_driveOpenLoopDemand = d.getRight();

	}


    public synchronized void updateTrajectoryFollower(double timestamp) {

    }

    public synchronized void updatePathVelocitySetpoint(DriveSignal d) {

        final double max_desired = Math.max(Math.abs(d.getLeft()), Math.abs(d.getRight()));
		final double scale = max_desired > TechConstants.kDriveMaxVelocity ? TechConstants.kDriveMaxVelocity / max_desired : 1.0;
        
        //leftFront.set(d.getLeft() * scale, ControlType.kSmartVelocity);
        //rightFront.set(d.getRight() * scale, ControlType.kSmartVelocity);

        //rightFront.set( (d.getRight() * scale) / 4.0 );
        rightFront.set(d.getRight() / 2.0);
        leftFront.set(d.getLeft() / 2.0);


        SmartDashboard.putNumber("Right Velocity demand", d.getRight());
        SmartDashboard.putNumber("Left Velocity demand", d.getLeft());

    }

    public synchronized void setWantDriveTrajectory(Trajectory trajectory, boolean reversed) {
        
        if (mCurrentTrajectory != trajectory || mControlMode != DriveControlState.PATH_FOLLOWING) {
            setControlMode(DriveControlState.PATH_FOLLOWING);
            mCurrentTrajectory = trajectory;
			// setControlMode(DriveControlState.PATH_FOLLOWING);
			// PathFollowerRobotState.getInstance().resetDistanceDriven();
			// mPathFollower = new PathFollower(path, reversed,
			// 		new PathFollower.Parameters(
			// 				new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
			// 						Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
			// 				Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
			// 				Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
			// 				Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
			// 				Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
			// 				Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
			// 				Constants.kPathStopSteeringDistance));

			// mCurrentPath = path;
		} else {
		}

	}

    public synchronized boolean isDoneWithTrajectory() {
        return mController.atReference();
    }


    public synchronized void setControlMode(DriveControlState controlMode) {
		if (controlMode != mControlMode) {
			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}
	}


    public synchronized Rotation2d getRotation() {
        //return mNavXBoard.getYaw();
        return Rotation2d.fromDegrees(mNavXBoard.getAngle());
    }

    public synchronized double getLeftDistanceMeters() {
        return leftFront.getEncoder().getPosition();
    }

    public synchronized double getRightDistanceMeters() {
        return rightFront.getEncoder().getPosition();
    }

    public synchronized DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftFront.getEncoder().getVelocity(), rightFront.getEncoder().getVelocity());
    }


    private synchronized void updateAutoAiming() {

        SmartDashboard.putNumber("Guessed Angle", mHubTargetingAngleGuess);

    }


    public synchronized void setHubAimingGuess(double guess) {
        if (guess != mHubTargetingAngleGuess) {
			try {
				_subsystemMutex.lock();
                mHubTargetingAngleGuess = guess;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}

    }

    public synchronized double getHubAimingGuess() {
        return mHubTargetingAngleGuess;
    }

    public boolean isAimedAtHub() {
        return rotationalPIDF.onTarget(0.05);
    }


    @Override
    public void init() {

        leftFront.restoreFactoryDefaults();
        rightFront.restoreFactoryDefaults();

        rightFront.setInverted(true);
        rightHind.setInverted(true);

        leftFront.setInverted(false);
        leftHind.setInverted(false);

        leftFront.getEncoder().setPositionConversionFactor( (0.1524 * Math.PI) / 10.71 );
        rightFront.getEncoder().setPositionConversionFactor( (0.1524 * Math.PI) / 10.71 );

        leftFront.getEncoder().setVelocityConversionFactor( (0.1524 * Math.PI) / 60.0 / 10.71 );
        rightFront.getEncoder().setVelocityConversionFactor( (0.1524 * Math.PI) / 60.0 / 10.71 );

        leftFront.setIdleMode(IdleMode.kCoast);
		rightFront.setIdleMode(IdleMode.kCoast);

		leftHind.setIdleMode(IdleMode.kCoast);
		rightHind.setIdleMode(IdleMode.kCoast);


		leftFront.setSmartCurrentLimit(35);
		rightFront.setSmartCurrentLimit(35);

		leftHind.setSmartCurrentLimit(35);
		rightHind.setSmartCurrentLimit(35);


		leftFront.setOpenLoopRampRate(0.15);
		rightFront.setOpenLoopRampRate(0.15);

		leftHind.setOpenLoopRampRate(0.15);
		rightHind.setOpenLoopRampRate(0.15);


        rotationalPIDF.setPID(TechConstants.kDriveRotationalKp, TechConstants.kDriveRotationalKi, TechConstants.kDriveRotationalKd);
		rotationalPIDF.setSetpoint(0);


        //TODO: IF AT ALL POSSIBLE AVOID DEADBAND
        //lol just tune better bro
		//rotationalPIDF.setDeadband(TechConstants.llTargetingDeadband);


        boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

            setSucceeded &= leftFront.getEncoder().setMeasurementPeriod(20) == REVLibError.kOk;

            setSucceeded &= rightFront.getEncoder().setMeasurementPeriod(20) == REVLibError.kOk;

		} while(!setSucceeded && retryCounter++ < TechConstants.kSparkMaxRetryCount);

        setSucceeded &= SparkHelper.setPIDGains(leftFront, 0, TechConstants.kDriveVelocityKp, TechConstants.kDriveVelocityKi, TechConstants.kDriveVelocityKd, TechConstants.kDriveVelocityKf, TechConstants.kDriveVelocityRampRate, TechConstants.kDriveVelocityIZone);
        setSucceeded &= SparkHelper.setPIDGains(leftFront, 1, TechConstants.kDriveVelocityKp, TechConstants.kDriveVelocityKi, TechConstants.kDriveVelocityKd, TechConstants.kDriveVelocityKf, TechConstants.kDriveVelocityRampRate, TechConstants.kDriveVelocityIZone);
        setSucceeded &= SparkHelper.setPIDGains(rightFront, 0, TechConstants.kDriveVelocityKp, TechConstants.kDriveVelocityKi, TechConstants.kDriveVelocityKd, TechConstants.kDriveVelocityKf, TechConstants.kDriveVelocityRampRate, TechConstants.kDriveVelocityIZone);
        setSucceeded &= SparkHelper.setPIDGains(rightFront, 1, TechConstants.kDriveVelocityKp, TechConstants.kDriveVelocityKi, TechConstants.kDriveVelocityKd, TechConstants.kDriveVelocityKf, TechConstants.kDriveVelocityRampRate, TechConstants.kDriveVelocityIZone);

		setSucceeded &= leftFront.getPIDController().setSmartMotionAccelStrategy(TechConstants.kDriveAccelStrategy, 0) ==  REVLibError.kOk;
		setSucceeded &= rightFront.getPIDController().setSmartMotionAccelStrategy(TechConstants.kDriveAccelStrategy, 0) == REVLibError.kOk;
		
		setSucceeded &= leftFront.getPIDController().setSmartMotionMaxAccel(TechConstants.kDriveMaxAccel, 0) ==  REVLibError.kOk;
		setSucceeded &= rightFront.getPIDController().setSmartMotionMaxAccel(TechConstants.kDriveMaxAccel, 0) == REVLibError.kOk;

		setSucceeded &= leftFront.getPIDController().setSmartMotionMaxVelocity(TechConstants.kDriveMaxVelocity, 0) ==  REVLibError.kOk;
		setSucceeded &= rightFront.getPIDController().setSmartMotionMaxVelocity(TechConstants.kDriveMaxVelocity, 0) == REVLibError.kOk;

        leftFront.selectProfileSlot(0, 0);
        rightFront.selectProfileSlot(0, 0);
        
    }

    @Override
    public void subsystemHome() {
        leftFront.resetEncoder();
        rightFront.resetEncoder();
        mNavXBoard.reset();
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }
    
    public enum DriveControlState {
        PATH_FOLLOWING,
        DRIVER_CONTROL,
        AUTO_AIMING,
        DISABLED,
        OPEN_LOOP;
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }


    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		public double left_driveOpenLoopDemand = 0;
		public double right_driveOpenLoopDemand = 0;

		public double shooter_guess = 0;
	}

}
