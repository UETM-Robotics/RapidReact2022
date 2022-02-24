package frc.robot.subsystems;

import java.net.http.HttpClient.Redirect;
import java.util.concurrent.locks.ReentrantLock;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.RobotState;
import frc.robot.Utilities.Constants;

public class Shooter extends Subsystem implements CustomSubsystem {


    private static Shooter instance = new Shooter();

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private Controllers controllers = Controllers.getInstance();

    public static Shooter getInstance() {
        return instance;
    }

    private final SparkMaxU shooterMotor;
    private final SparkMaxU hoodMotor;

    private ShooterControlMode mShooterControlMode = ShooterControlMode.SMART_VELOCITY;

    private PeriodicIO mPeriodicIO;

    private final ElapsedTimer loopTimer = new ElapsedTimer();
	private final ElapsedTimer shooter_dt = new ElapsedTimer();

    private Shooter() {

        shooterMotor = controllers.getShooterMotor();
        hoodMotor = controllers.getHoodMotor();

        mPeriodicIO = new PeriodicIO();

    }
    
    public void hood(){
        hoodMotor.set(0.5);
    }


    Loop mloop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            synchronized(Shooter.this) {
                zeroSensors();
            }
        }

        @Override
        public void onStart(double timestamp) {
            synchronized(Shooter.this) {
                shooter_dt.start();
            }
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            loopTimer.start();
            
            synchronized(Shooter.this) {

                switch(mShooterControlMode) {
                    case DISABLED:
                        shooterMotor.set(0);
                        break;
                    case OPEN_LOOP:
                        shooterMotor.set( Math.min(Math.max(mPeriodicIO.shooter_setpoint_rpm, -1), 1));
                        break;
                    case SMART_VELOCITY:
                        shooterMotor.set(mPeriodicIO.shooter_setpoint_rpm, ControlType.kSmartVelocity);  
                        break;
                    case VELOCITY:
                        shooterMotor.set(mPeriodicIO.shooter_setpoint_rpm, ControlType.kVelocity);
                        break;
                    default:
                        shooterMotor.set(0);
                        DriverStation.reportError("Anomaly occurred setting shooter power", false);
                        break;

                }

            }            
            
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }
        
    };

    public synchronized void setShooterVelocity(double shooterVelocity) {
		mPeriodicIO.shooter_setpoint_rpm = shooterVelocity;
	}

    public void setShooterVelocityChango(double shooterVelocity) {
        shooterMotor.set(ControlType.kVelocity, shooterVelocity, 0);
        //shooterMotor.set(shooterVelocity);
    }

    public double getvel() {
        return shooterMotor.getEncoder().getVelocity();
    }

    public synchronized void setShooterControlMode( ShooterControlMode controlMode ) {

        if (controlMode != mShooterControlMode) {
			try {
				_subsystemMutex.lock();
				mShooterControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}

    }

    public Pose2d getLatestFieldToTurretPose() {
		return RobotState.getInstance().getLatestFieldToVehicle().getValue().transformBy(getLatestVehicleToTurretPose());
	}

    public Pose2d getLatestVehicleToTurretPose() {
		return new Pose2d(Constants.kVehicleToTurret.getTranslation(),
							Rotation2d.fromDegrees(0));
	}


    @Override
    public void init() {
        
        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.setOpenLoopRampRate(0.2);
        shooterMotor.setSmartCurrentLimit(70);

        hoodMotor.setIdleMode(IdleMode.kBrake);
        hoodMotor.setOpenLoopRampRate(0.15);
        hoodMotor.setSmartCurrentLimit(45);
        

        int retryCounter = 0;
        boolean setSucceeded;

        do {

            setSucceeded = true;

            setSucceeded &= shooterMotor.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;

            setSucceeded &= SparkHelper.setPIDGains(hoodMotor, 0, Constants.kHoodVelocityKp, Constants.kHoodVelocityKi, Constants.kHoodVelocityKd, Constants.kHoodVelocityKf, Constants.kHoodVelocityClosedLoopRampRate, Constants.kHoodVelocityIZone);
            setSucceeded &= hoodMotor.getPIDController().setSmartMotionAccelStrategy(Constants.kHoodAccelStrategy, 0) == REVLibError.kOk;
            setSucceeded &= hoodMotor.getPIDController().setSmartMotionMaxVelocity(Constants.kHoodMaxVelocity, 0) == REVLibError.kOk;

            setSucceeded &= SparkHelper.setPIDGains(shooterMotor, 0, Constants.kShooterVelocityKp, Constants.kShooterVelocityKi, Constants.kShooterVelocityKd, Constants.kShooterVelocityKf, Constants.kShooterVelocityClosedLoopRampRate, Constants.kShooterVelocityIZone);
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionAccelStrategy(Constants.kShooterAccelStrategy, 0) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionMaxVelocity(Constants.kShooterMaxVelocity, 0) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionMaxAccel(Constants.kShooterMaxAccel, 0) == REVLibError.kOk;

            
        } while(retryCounter++ < 3 && !setSucceeded);

        if(retryCounter == 3) {
            DriverStation.reportError("Error Initializing Shooter", false);
        }
    }

    @Override
    public void subsystemHome() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mloop);
    }

    @Override
    public void stop() {
        shooterMotor.set(0);
    }


    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {

		public double shooter_velocity_rpm = 0;
		public double shooter_setpoint_rpm = 0;

		// Outputs
		public double shooter_loop_time = 0;
	}
    

    public enum ShooterControlMode {
		OPEN_LOOP,
		VELOCITY,
        SMART_VELOCITY,
		DISABLED;
	}

    public enum HoodControlMode {
		OPEN_LOOP,
		POSITION,
		DISABLED;
	}
}
