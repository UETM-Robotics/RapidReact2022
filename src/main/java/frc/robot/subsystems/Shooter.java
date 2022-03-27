package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;

public class Shooter extends Subsystem implements CustomSubsystem {

    private static final Shooter instance = new Shooter();
    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    public static Shooter getInstance() {
        return instance;
    }

    private Shooter() {
        shooterMotor = Controllers.getInstance().getShooterMotor();
        hoodMotor = Controllers.getInstance().getHoodMotor();
        beltIndexerMotor = Controllers.getInstance().getBeltIndexerMotor();

        mPeriodicIO = new PeriodicIO();
    }


    private final SparkMaxU shooterMotor;
    private final SparkMaxU hoodMotor;
    private final SparkMaxU beltIndexerMotor;

    private ShooterControlMode mShooterControlMode = ShooterControlMode.DISABLED;
    private HoodControlMode mHoodControlMode = HoodControlMode.DISABLED;
    private BeltIndexerControlMode mBeltIndexerControlMode = BeltIndexerControlMode.DISABLED;


    private PeriodicIO mPeriodicIO;
    

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
            synchronized (Shooter.this) {

                switch(mShooterControlMode) {
                    case DISABLED:

                        shooterMotor.set(0);

                        break;
                    case INTERPOLATING:

                        shooterMotor.set( interpolateShooter(), ControlType.kSmartVelocity );

                        break;
                    case OPEN_LOOP:

                        shooterMotor.set( Math.min(Math.max(mPeriodicIO.shooter_setpoint_rpm, -1), 1));

                        break;
                    case VELOCITY:

                        shooterMotor.set(mPeriodicIO.shooter_setpoint_rpm, ControlType.kSmartVelocity);

                        break;
                    default:
                        shooterMotor.set(0);
                        DriverStation.reportError("Anomaly in setting Shooter Control Mode", true);
                        break;

                }

                switch(mHoodControlMode) {
                    case DISABLED:
                    
                        hoodMotor.set(0);

                        break;
                    case INTERPOLATING:

                        hoodMotor.set( interpolateHood(), ControlType.kSmartMotion );

                        break;
                    case POSITION:

                        hoodMotor.set(mPeriodicIO.hood_position_rotations, ControlType.kSmartMotion);

                        break;
                    case OPEN_LOOP:

                        hoodMotor.set( Math.min(Math.max(mPeriodicIO.hood_position_rotations, -1), 1));

                        break;
                    default:
                        hoodMotor.set(0);
                        DriverStation.reportError("Anomaly in setting Hood Control Mode", true);
                        break;
                }

                switch(mBeltIndexerControlMode) {
                    case DISABLED:

                        beltIndexerMotor.set(0);

                        break;
                    case ENABLED:

                        beltIndexerMotor.set(0.8);

                        break;
                    case SELF_AWARE:
                        break;
                    default:
                        beltIndexerMotor.set(0);
                        DriverStation.reportError("Anomaly in setting Belt Indexer Control Mode", true);
                        break;

                }

            }
        }

        @Override
        public void onStop(double timestamp) {
            // TODO Auto-generated method stub
            
        }
        
    };


    private synchronized double interpolateShooter() {
        return 0;
    }

    private synchronized double interpolateHood() {
        return 0;
    }


    public synchronized void setShooterVelocitySetpoint(double shooterVelocity) {
        mPeriodicIO.shooter_setpoint_rpm = shooterVelocity;
    }

    public synchronized double getShooterVelocitySetpoint() {
        return mPeriodicIO.shooter_setpoint_rpm;
    }

    public synchronized double getShooterVelocity() {
        return shooterMotor.getEncoder().getVelocity();
    }


    public synchronized void setHoodPositionSetpoint(double hoodPosition) {
        mPeriodicIO.hood_position_rotations = hoodPosition;
    }


    public synchronized void setShooterControlMode(ShooterControlMode controlMode) {
        if (controlMode != mShooterControlMode) {
			try {
				_subsystemMutex.lock();
				mShooterControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}
    }

    public synchronized void setHoodControlMode(HoodControlMode controlMode) {
        if (controlMode != mHoodControlMode) {
			try {
				_subsystemMutex.lock();
				mHoodControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}
    }

    public synchronized void setBeltIndexerControlMode(BeltIndexerControlMode controlMode) {
        if (controlMode != mBeltIndexerControlMode) {
			try {
				_subsystemMutex.lock();
				mBeltIndexerControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}
    }


    @Override
    public void init() {
        
        shooterMotor.setIdleMode(IdleMode.kCoast);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        beltIndexerMotor.setIdleMode(IdleMode.kCoast);

        shooterMotor.setSmartCurrentLimit(60);
        shooterMotor.setOpenLoopRampRate(0.2);

        hoodMotor.setSmartCurrentLimit(30);
        hoodMotor.setOpenLoopRampRate(0.2);

        beltIndexerMotor.setSmartCurrentLimit(30);
        beltIndexerMotor.setOpenLoopRampRate(0.2);


        int retryCounter = 0;
        boolean setSucceeded;

        do {

            setSucceeded = true;

            setSucceeded &= shooterMotor.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;
            setSucceeded &= hoodMotor.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;

            setSucceeded &= SparkHelper.setPIDGains(hoodMotor, 0, TechConstants.kHoodVelocityKp, TechConstants.kHoodVelocityKi, TechConstants.kHoodVelocityKd, TechConstants.kHoodVelocityKf, TechConstants.kHoodVelocityClosedLoopRampRate, TechConstants.kHoodVelocityIZone);
            setSucceeded &= hoodMotor.getPIDController().setSmartMotionAccelStrategy(TechConstants.kHoodAccelStrategy, 0) == REVLibError.kOk;
            setSucceeded &= hoodMotor.getPIDController().setSmartMotionMaxVelocity(TechConstants.kHoodMaxVelocity, 0) == REVLibError.kOk;
            setSucceeded &= hoodMotor.getPIDController().setSmartMotionMaxAccel(TechConstants.kHoodMaxAccel, 0) == REVLibError.kOk;
            setSucceeded &= hoodMotor.getPIDController().setSmartMotionAllowedClosedLoopError(TechConstants.kHoodSmartMotionAllowedClosedLoopError, 0) == REVLibError.kOk;

            setSucceeded &= SparkHelper.setPIDGains(shooterMotor, 0, TechConstants.kShooterVelocityKp, TechConstants.kShooterVelocityKi, TechConstants.kShooterVelocityKd, TechConstants.kShooterVelocityKf, TechConstants.kShooterVelocityClosedLoopRampRate, TechConstants.kShooterVelocityIZone);
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionAccelStrategy(TechConstants.kShooterAccelStrategy, 0) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionMaxVelocity(TechConstants.kShooterMaxVelocity, 0) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionMaxAccel(TechConstants.kShooterMaxAccel, 0) == REVLibError.kOk;

            
        } while(retryCounter++ < 3 && !setSucceeded);

    }

    @Override
    public void subsystemHome() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }


    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {

		public double shooter_setpoint_rpm = 0;

        public double hood_position_rotations = 0;

		// Outputs
		public double shooter_loop_time = 0;
	}
    

    public enum ShooterControlMode {
        INTERPOLATING,
        OPEN_LOOP,
        VELOCITY,
        DISABLED;
    }

    public enum HoodControlMode {
        INTERPOLATING,
        OPEN_LOOP,
        POSITION,
        DISABLED;
    }

    public enum BeltIndexerControlMode {
        SELF_AWARE,
        ENABLED,
        DISABLED;
    }

}
