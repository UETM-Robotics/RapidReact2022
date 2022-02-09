package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.ControlType;

import frc.robot.Utilities.CachedValue;
import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.DiagnosticMessage;
import frc.robot.Utilities.DriveMotorValues;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;

public class Shooter extends Subsystem implements CustomSubsystem {


    private static Shooter instance = new Shooter();

    private final Vision mVision = Vision.getInstance();

    private final Controllers controllers = Controllers.getInstance();


    private final SparkMaxU mBeltTransporterMotor;
    private final SparkMaxU mShooterMotor;
    private final SparkMaxU mHoodMotor;

    private HoodControlMode mhoodControlMode = HoodControlMode.POSITION;
    private ShooterControlMode mshooterControlMode = ShooterControlMode.SMART_VELOCITY;

    private PeriodicIO mPeriodicIO;

    private final ElapsedTimer loopTimer = new ElapsedTimer();
    private final ElapsedTimer shooter_dt = new ElapsedTimer();


    Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            synchronized (Shooter.this) {
                zeroSensors();
            }
        }

        @Override
        public void onStart(double timestamp) {
            shooter_dt.start();
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {

            switch(mhoodControlMode) {
                case DISABLED:
                    mHoodMotor.set(0.0);
                    break;
                case OPEN_LOOP:
                    mHoodMotor.set( Math.min( Math.max(mPeriodicIO.hood_position_deg, -1), 1) );
                    break;
                case POSITION:
                    mHoodMotor.set(mPeriodicIO.hood_setpoint_deg, ControlType.kSmartMotion);
                    break;
                default:
                    mHoodMotor.set(0.0);
                    System.out.println("Error Setting Hood Control Mode, Defaulting...");
                    break;

            }

            switch(mshooterControlMode) {
                case DISABLED:
                    mShooterMotor.set(0.0);
                    break;
                case OPEN_LOOP:
                    mShooterMotor.set( Math.min( Math.max(mPeriodicIO.shooter_setpoint_rpm, -1), 1 ) );
                    break;
                case SMART_VELOCITY:
                    mShooterMotor.set( mPeriodicIO.shooter_setpoint_rpm, ControlType.kSmartVelocity );
                    break;
                case VELOCITY:
                    mShooterMotor.set( getAccelFilteredShooterVelocity(), ControlType.kVelocity );
                    break;
                default:
                    mShooterMotor.set(0.0);
                    break;

            }

            readPeriodicInputs();

        }

        @Override
        public void onStop(double timestamp) {
            
            mHoodMotor.set(0.0);
		    mShooterMotor.set(0.0);
            
        }

    };

    
    private Shooter() {

        mPeriodicIO = new PeriodicIO();

        //Diameter - 
        //Radius - 
        //Teeth -
        mHoodMotor = controllers.getHoodMotor();


        mShooterMotor = controllers.getShooterMotor();

        mBeltTransporterMotor = controllers.getShooterMotor();

    }

    public double getAccelFilteredShooterVelocity() {
		double diffErr = mPeriodicIO.shooter_setpoint_rpm - mPeriodicIO.shooter_velocity_rpm;
		if (Constants.kShooterMaxAccel == 0)
			return mPeriodicIO.shooter_setpoint_rpm;
		double outputVel = mPeriodicIO.shooter_setpoint_rpm + Math.min(Math.abs(diffErr), (Constants.kShooterMaxAccel * shooter_dt.hasElapsed())) * Math.copySign(1.0, diffErr);
		shooter_dt.start();
		return outputVel;
	}

    public static Shooter getInstance() {
        return instance;
    }

    @Override
	public synchronized void writePeriodicOutputs() {
		loopTimer.start();
		mPeriodicIO.turret_loop_time += loopTimer.hasElapsed();
	}


    @Override
    public void init() {
        // TODO : INITIALIZE HOOD MOTOR
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void subsystemHome() {
        
    }


    @Override
	public void registerEnabledLoops(Looper in) {
		in.register(mLoop);
	}

    @Override
    public void zeroSensors() {
        mHoodMotor.getEncoder().setPosition(0.0);
        setHoodPosition(0);

        if(mhoodControlMode == HoodControlMode.POSITION) {
            mHoodMotor.set(0.0);
        }
    }

    public synchronized void setHoodPosition(double hoodPosition) {
		mPeriodicIO.hood_setpoint_deg = hoodPosition;
	}

    public synchronized void setShooterVelocity(double shooterVelocity) {
		mPeriodicIO.shooter_setpoint_rpm = shooterVelocity;
	}

    public synchronized void setHoodControlMode(HoodControlMode hoodControlMode) {
		if (mhoodControlMode != hoodControlMode)
			mhoodControlMode = hoodControlMode;
	}

	public synchronized void setShooterControlMode(ShooterControlMode shooterControlMode) {
		if (mshooterControlMode != shooterControlMode)
			mshooterControlMode = shooterControlMode;
	}

    public double getShooterVelocity() {
        return mPeriodicIO.shooter_setpoint_rpm;
    }

    public enum HoodControlMode {
		OPEN_LOOP,
		POSITION,
		DISABLED;
	}

    public enum ShooterControlMode {
		OPEN_LOOP,
		VELOCITY,
        SMART_VELOCITY,
		DISABLED;
	}

    @Override
	public synchronized void readPeriodicInputs() {
		loopTimer.start();
		mPeriodicIO.turret_loop_time = loopTimer.hasElapsed();
		mPeriodicIO.shooter_velocity_rpm = mShooterMotor.getVelocity();
		mPeriodicIO.hood_position_deg = mHoodMotor.getPosition();
	}

    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {
		//Making members public here will automatically add them to logs
		// INPUTS

		public double shooter_velocity_rpm;
		public double shooter_setpoint_rpm;

		public double hood_position_deg;
		public double hood_setpoint_deg;
		public boolean hood_reset;

		// Outputs
		public double turret_loop_time;
	}
    
}
