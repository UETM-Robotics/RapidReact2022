package frc.robot.subsystems;

import java.net.http.HttpClient.Redirect;
import java.util.concurrent.locks.ReentrantLock;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;

import frc.robot.Utilities.Constants;

public class Shooter extends Subsystem implements CustomSubsystem {


    private static Shooter instance = new Shooter();

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private Controllers controllers = Controllers.getInstance();

    public static Shooter getInstance() {
        return instance;
    }

    private final SparkMaxU shooterMotor;
    private final SparkMaxU beltTransporterMotor;

    private ShooterControlMode mShooterControlMode = ShooterControlMode.SMART_VELOCITY;

    private PeriodicIO mPeriodicIO;

    private final ElapsedTimer loopTimer = new ElapsedTimer();
	private final ElapsedTimer shooter_dt = new ElapsedTimer();

    private Shooter() {

        shooterMotor = controllers.getShooterMotor();
        beltTransporterMotor = controllers.getBeltTransporterMotor();

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
                        shooterMotor.set(mPeriodicIO.shooter_setpoint_rpm);
                        break;
                    case VELOCITY:
                        shooterMotor.set(mPeriodicIO.shooter_setpoint_rpm);
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


    @Override
    public void init() {
        
        shooterMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.setOpenLoopRampRate(0.2);
        shooterMotor.setSmartCurrentLimit(70);

        beltTransporterMotor.setIdleMode(IdleMode.kCoast);
        beltTransporterMotor.setOpenLoopRampRate(0.5);
        beltTransporterMotor.setSmartCurrentLimit(40);

        int retryCounter = 0;
        boolean setSucceeded;

        do {

            setSucceeded = true;

            setSucceeded &= shooterMotor.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;
            setSucceeded &= beltTransporterMotor.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;


            setSucceeded &= beltTransporterMotor.setOpenLoopRampRate(0.2) == REVLibError.kOk;

            setSucceeded &= SparkHelper.setPIDGains(shooterMotor, 0, Constants.kShooterVelocityKp, Constants.kShooterVelocityKi, Constants.kShooterVelocityKd, Constants.kShooterVelocityKf, Constants.kShooterVelocityClosedLoopRampRate, Constants.kShooterVelocityIZone);
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionAccelStrategy(Constants.kShooterAccelStrategy, 0) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionMaxVelocity(Constants.kShooterMaxVelocity, 0) == REVLibError.kOk;

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
        
    }

    @Override
    public void stop() {
        shooterMotor.set(0);
    }


    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {

		public double shooter_velocity_rpm;
		public double shooter_setpoint_rpm;

		// Outputs
		public double shooter_loop_time;
	}
    

    public enum ShooterControlMode {
		OPEN_LOOP,
		VELOCITY,
        SMART_VELOCITY,
		DISABLED;
	}
}
