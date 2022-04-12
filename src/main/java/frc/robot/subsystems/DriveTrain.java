package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.DriveSignal;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.TrajectoryFollowing.DifferentialDriveWheelSpeeds;


//DriveTrain class for a 6 wheel tank drive robot
public class DriveTrain extends Subsystem implements CustomSubsystem {


    private static final DriveTrain instance = new DriveTrain();

    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    
    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private boolean mPrevBrakeModeVal = false;


    public static DriveTrain getInstance() {
        return instance;
    }


    
    private DriveTrain() {

        leftFront = Controllers.getInstance().getLeftFrontDriveMotor();
        leftHind = Controllers.getInstance().getLeftHindDriveMotor();

        rightFront = Controllers.getInstance().getRightFrontDriveMotor();
        rightHind = Controllers.getInstance().getRightHindDriveMotor();

        gyro = Controllers.getInstance().getGyro();
    }


    private final SparkMaxU leftFront, rightFront;
    private final SparkMaxU leftHind , rightHind;

    private AHRS gyro;

    private ControllerU driverController;
    private double mThrottle, mTurn;

    
    private DriveControlState mControlMode = DriveControlState.DISABLED;


    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            subsystemHome();
        }

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            
            synchronized(DriveTrain.this) {

                switch(mControlMode) {
                    case DISABLED:

                        leftFront.set(0);
                        rightFront.set(0);

                        break;
                    case DRIVER_CONTROL:

                        mThrottle = -driverController.getNormalizedAxis(1, TechConstants.kJoystickDeadband);
                        mTurn = driverController.getNormalizedAxis(2, TechConstants.kJoystickDeadband) * 0.4;
                        leftFront.set(Math.max(Math.min(mThrottle + mTurn, 1), -1));
                        rightFront.set( Math.max(Math.min(mThrottle - mTurn, 1), -1));

                        break;
                    case OPEN_LOOP:

                        leftFront.set(mPeriodicIO.left_driveOpenLoopDemand);
                        rightFront.set(mPeriodicIO.right_driveOpenLoopDemand);

                        break;
                    case PATH_FOLLOWING:
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

		mPeriodicIO.left_driveOpenLoopDemand = d.getLeft();
		mPeriodicIO.right_driveOpenLoopDemand = d.getRight();

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
        return Rotation2d.fromDegrees(gyro.getAngle());
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


    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void init() {

        leftFront.restoreFactoryDefaults();
        rightFront.restoreFactoryDefaults();

        rightFront.setInverted(true);
        rightHind.setInverted(true);

        leftFront.setInverted(false);
        leftHind.setInverted(false);

        leftFront.getEncoder().setPositionConversionFactor( TechConstants.kRotationsToMetersConversionFactor );
        rightFront.getEncoder().setPositionConversionFactor( TechConstants.kRotationsToMetersConversionFactor );

        leftFront.getEncoder().setVelocityConversionFactor( TechConstants.kRPMtoMetersPerSecondConversionFactor );
        rightFront.getEncoder().setVelocityConversionFactor( TechConstants.kRPMtoMetersPerSecondConversionFactor );

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
        gyro.reset();
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		public double left_driveOpenLoopDemand = 0;
		public double right_driveOpenLoopDemand = 0;

		public double shooter_guess = 0;
	}
    
    public enum DriveControlState {
        DRIVER_CONTROL,
        OPEN_LOOP,
        PATH_FOLLOWING,
        DISABLED;
    }
}
