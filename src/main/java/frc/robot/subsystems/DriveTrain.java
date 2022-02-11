package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.DriveControlState;
import frc.robot.Utilities.DriveMotorValues;
import frc.robot.Utilities.Drivers.NavX;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.TrajectoryFollowingMotion.Kinematics;
import frc.robot.Utilities.TrajectoryFollowingMotion.Path;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathFollower;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathFollowerRobotState;
import frc.robot.Utilities.TrajectoryFollowingMotion.RigidTransform2d;
import frc.robot.Utilities.TrajectoryFollowingMotion.Rotation2d;
import frc.robot.Utilities.TrajectoryFollowingMotion.Twist2d;
import frc.robot.Utilities.TrajectoryFollowingMotion.Util;
import frc.robot.Utilities.TrajectoryFollowingMotion.Lookahead;

public class DriveTrain extends SubsystemBase implements CustomSubsystem{

    private static DriveTrain instance = null;


    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    public SparkMaxU leftFront, rightFront;
    public SparkMaxU leftHind , rightHind;

    private NavX mNavXBoard;

    private DriveControlState mControlMode;
    private boolean mPrevBrakeModeVal;

    private Path mCurrentPath = null;
    private PathFollower mPathFollower;

    private PathFollowerRobotState mRobotState = PathFollowerRobotState.getInstance();

    public static DriveTrain getInstance() {
        if(instance == null) {
            instance = new DriveTrain();
        }

        return instance;
    }

    private DriveTrain() {

        Controllers robotControllers = Controllers.getInstance();

        leftFront = robotControllers.getLeftFrontDrive();
        leftHind = robotControllers.getLeftHindDrive();

        rightFront = robotControllers.getRightFrontDrive();
        rightHind = robotControllers.getRightHindDrive();

        mNavXBoard = robotControllers.getNavX();
        
        


        mControlMode = DriveControlState.PATH_FOLLOWING;
        mPrevBrakeModeVal = false;

    }

    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            synchronized (DriveTrain.this) {
                subsystemHome();
				mNavXBoard.zeroYaw();
				mNavXBoard.reset();


				leftFront.getEncoder().setPosition(0);
				rightFront.getEncoder().setPosition(0);
            }
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (DriveTrain.this) {
                setDriveOpenLoop(DriveMotorValues.NEUTRAL);
                setBrakeMode(false);
                setDriveVelocity(new DriveMotorValues(0, 0));
            }
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (DriveTrain.this) {
                switch (mControlMode) {
					case OPEN_LOOP:
						break;
					case VELOCITY:
						break;
					case TURN_TO_HEADING:
						//updateTurnToHeading(timestamp);
						break;
					case PATH_FOLLOWING:
						if (mPathFollower != null) {
							updatePathFollower(timestamp);
							//mCSVWriter.add(mPathFollower.getDebug());
						}
						break;
					case AUTO_AIMING:
						break;
					default:
						//ConsoleReporter.report("Unexpected drive control state: " + mControlMode);
						break;
				}
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    };

    @Override
    public void subsystemHome() {
		mNavXBoard.zeroYaw();
		mNavXBoard.reset();


		leftFront.getEncoder().setPosition(0);
		rightFront.getEncoder().setPosition(0);
	}

    public void setControlMode(DriveControlState controlMode) {
		if (controlMode != mControlMode) {
			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}
	}

	public DriveControlState getDriveControlState() {
		return mControlMode;
	}

    public synchronized void setDriveOpenLoop(DriveMotorValues d) {
        setControlMode(DriveControlState.OPEN_LOOP);

        leftFront.set(d.leftDrive);
        rightFront.set(d.rightDrive);
	}

    public void setBrakeMode(boolean brakeMode) {
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

    public synchronized void setDriveVelocity(DriveMotorValues d) {
		setDriveVelocity(d, true);
	}

	public synchronized void setDriveVelocity(DriveMotorValues d, boolean autoChangeMode) {
		if (autoChangeMode)
			setControlMode(DriveControlState.VELOCITY);
        
        leftFront.set(Util.convertRPMToNativeUnits(d.leftDrive), ControlType.kVelocity);
        rightFront.set(Util.convertRPMToNativeUnits(d.rightDrive), ControlType.kVelocity);

	}

    private void updatePathFollower(double timestamp) {
		RigidTransform2d robot_pose = mRobotState.getLatestFieldToVehicle().getValue();
		Twist2d command = mPathFollower.update(timestamp, robot_pose,
				PathFollowerRobotState.getInstance().getDistanceDriven(), PathFollowerRobotState.getInstance().getPredictedVelocity().dx);

		if (!mPathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			updatePathVelocitySetpoint(setpoint.left, setpoint.right);

			SmartDashboard.putNumber("left setpoint", setpoint.left);
			SmartDashboard.putNumber("right setpoint", setpoint.right);

			//ConsoleReporter.report(mPathFollower.getDebug());
			//ConsoleReporter.report("Left2Cube: " + inchesPerSecondToRpm(setpoint.left) + ", Right2Cube: " + inchesPerSecondToRpm(setpoint.right));
			//ConsoleReporter.report("Left2Cube Actual: " + Util.convertNativeUnitsToRPM(mLeftMaster.getSelectedSensorVelocity(0)) + ", Right2Cube Actual: " + Util.convertNativeUnitsToRPM(mRightMaster.getSelectedSensorVelocity(0)));
		} else {
			updatePathVelocitySetpoint(0, 0);
			setControlMode(DriveControlState.VELOCITY);
		}
	}

	private void updatePathVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		final double max_desired = Math.max(Math.abs(left_inches_per_sec), Math.abs(right_inches_per_sec));
		final double scale = max_desired > Constants.kDriveHighGearMaxSetpoint ? Constants.kDriveHighGearMaxSetpoint / max_desired : 1.0;

        // leftFront.set(Util.convertRPMToNativeUnits(inchesPerSecondToRpm(left_inches_per_sec * scale)), ControlType.kVelocity);
        // rightFront.set(Util.convertRPMToNativeUnits(inchesPerSecondToRpm(right_inches_per_sec * scale)), ControlType.kVelocity);

		leftFront.set(left_inches_per_sec / 165.354);
		rightFront.set(right_inches_per_sec / 165.354);

		//ConsoleReporter.report("Requested Drive Velocity Left2Cube/Right2Cube: " + left_inches_per_sec + "/" + right_inches_per_sec);
		//ConsoleReporter.report("Actual Drive Velocity Left2Cube/Right2Cube: " + getLeftVelocityInchesPerSec() + "/" + getRightVelocityInchesPerSec());
	}

    private static double rotationsToInches(double rotations) {
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	}

    private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}

    private static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	}

    private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
	}

    public double getLeftDistanceInches() {
		return leftFront.getEncoder().getPosition();
	}

    public double getRightDistanceInches() {
		return rightFront.getEncoder().getPosition();
	}

    public synchronized Rotation2d getGyroAngle() {
		return mNavXBoard.getYaw();
	}

    public synchronized void setGyroAngle(Rotation2d angle) {
		mNavXBoard.reset();
		mNavXBoard.setAngleAdjustment(angle);
	}

	public void setBruh() {
		leftFront.set(0.05);
		rightFront.set(0.05);
	}

    public double getLeftVelocityInchesPerSec() { 
		return ( leftFront.getEncoder().getVelocity() * Math.PI * 6) / 60;
		//rpmToInchesPerSecond(Util.convertNativeUnitsToRPM(leftFront.getEncoder().getVelocity())); 
	}

	public double getRightVelocityInchesPerSec() { return ( rightFront.getEncoder().getVelocity() * Math.PI * 6) / 60; }

    public synchronized void setWantDrivePath(Path path, boolean reversed) {
		if (mCurrentPath != path || mControlMode != DriveControlState.PATH_FOLLOWING) {
			setControlMode(DriveControlState.PATH_FOLLOWING);
			PathFollowerRobotState.getInstance().resetDistanceDriven();
			mPathFollower = new PathFollower(path, reversed,
					new PathFollower.Parameters(
							new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead,
									Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed),
							Constants.kInertiaSteeringGain, Constants.kPathFollowingProfileKp,
							Constants.kPathFollowingProfileKi, Constants.kPathFollowingProfileKv,
							Constants.kPathFollowingProfileKffv, Constants.kPathFollowingProfileKffa,
							Constants.kPathFollowingMaxVel, Constants.kPathFollowingMaxAccel,
							Constants.kPathFollowingGoalPosTolerance, Constants.kPathFollowingGoalVelTolerance,
							Constants.kPathStopSteeringDistance));

			mCurrentPath = path;
		} else {
		}
	}

    public synchronized boolean isDoneWithPath() {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			return mPathFollower.isFinished();
		} else {
			if (mPathFollower != null)
				return mPathFollower.isFinished();
			else
				return true;
		}
	}

    public synchronized void forceDoneWithPath() {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			mPathFollower.forceFinish();
		} else {
		}
	}

	public synchronized boolean hasPassedMarker(String marker) {
		if (mControlMode == DriveControlState.PATH_FOLLOWING && mPathFollower != null) {
			return mPathFollower.hasPassedMarker(marker);
		} else {
			if (mPathFollower != null)
				return (mPathFollower.isFinished() || mPathFollower.hasPassedMarker(marker));
			else {
				//TODO: Test with false value
				return true;
			}
		}
	}

    @Override
    public void init() {
        rightFront.setInverted(true);
        rightHind.setInverted(true);

        leftFront.setInverted(false);
        leftHind.setInverted(false);

		leftFront.getEncoder().setVelocityConversionFactor(1);
		rightFront.getEncoder().setVelocityConversionFactor(1);

		leftFront.getEncoder().setPositionConversionFactor(1);
		rightFront.getEncoder().setPositionConversionFactor(1);

		leftFront.setIdleMode(IdleMode.kCoast);
		rightFront.setIdleMode(IdleMode.kCoast);

		leftHind.setIdleMode(IdleMode.kCoast);
		rightHind.setIdleMode(IdleMode.kCoast);

        boolean setSucceeded;
		int retryCounter = 0;

		do {
			setSucceeded = true;

            setSucceeded &= leftFront.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;

            setSucceeded &= rightFront.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;

		} while(!setSucceeded && retryCounter++ < Constants.kSparkMaxRetryCount);

        setSucceeded &= SparkHelper.setPIDGains(leftFront, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
        setSucceeded &= SparkHelper.setPIDGains(leftFront, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
        setSucceeded &= SparkHelper.setPIDGains(rightFront, 0, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);
        setSucceeded &= SparkHelper.setPIDGains(rightFront, 1, Constants.kDriveHighGearVelocityKp, Constants.kDriveHighGearVelocityKi, Constants.kDriveHighGearVelocityKd, Constants.kDriveHighGearVelocityKf, Constants.kDriveHighGearVelocityRampRate, Constants.kDriveHighGearVelocityIZone);

        leftFront.selectProfileSlot(0, 0);
        rightFront.selectProfileSlot(0, 0);
    }

    @Override
	public void registerEnabledLoops(Looper in) {
		in.register(mLoop);
	}
	
	public void reportSpeeds() {
		SmartDashboard.putNumber("left speed", leftFront.get());
		SmartDashboard.putNumber("right speed", rightFront.get());
	}

	public void reportPosition() {
		SmartDashboard.putNumber("left position", leftFront.getEncoder().getPosition());
		SmartDashboard.putNumber("right position", rightHind.getEncoder().getPosition());
	}
}
