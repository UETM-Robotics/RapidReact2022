package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.ShooterControlState;
import frc.robot.Utilities.ShooterMotorValues;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;

public class tempShooter implements CustomSubsystem
{

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private static tempShooter instance;


    public static tempShooter getInstance() {
        if(instance == null) {
            instance = new tempShooter();
        }

        return instance;
    }


    private SparkMaxU shooterMotor,
                      beltTransportMotor;
                      
    private ShooterControlState mControlMode;


    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            subsystemHome();
        }

        @Override
        public void onStart(double timestamp) {
            synchronized (tempShooter.this) {
                setShooterOpenLoop(ShooterMotorValues.NEUTRAL);
                setShooterVelocity( new ShooterMotorValues(0.0) );
            }
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            switch(mControlMode) {
                case OPEN_LOOP:
                    break;
                case SMART_VELOCITY:
                    break;
                case VELOCITY:
                    break;
                default:
                    break;

            }
        }

        @Override
        public void onStop(double timestamp) {
            
        }
        
    };


    private tempShooter() {

        Controllers controllers = Controllers.getInstance();
        

        shooterMotor = controllers.getShooterMotor();
        beltTransportMotor = controllers.getBeltTransportMotor();
        

        mControlMode = ShooterControlState.VELOCITY;
        
    }


    public void setControlMode(ShooterControlState controlMode) {
		if (controlMode != mControlMode) {
			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                System.out.println("FAILED TO SET SHOOTER CONTROL MODE");
			}
		}
	}

    public synchronized void setShooterOpenLoop(ShooterMotorValues s) {
        setControlMode(ShooterControlState.OPEN_LOOP);

        shooterMotor.set(s.magnitude);
	}

    public synchronized void setShooterVelocity(ShooterMotorValues s) {
        setShooterVelocity(s, true);
	}

	public synchronized void setShooterVelocity(ShooterMotorValues s, boolean autoChangeMode) {
		if (autoChangeMode)
			setControlMode(ShooterControlState.VELOCITY);
        
        shooterMotor.set(s.magnitude, ControlType.kVelocity);

	}

    public synchronized void setShooterSmartVelocity(ShooterMotorValues s, boolean autoChangeMode) {
        if (autoChangeMode)
			setControlMode(ShooterControlState.SMART_VELOCITY);
        
        shooterMotor.set(s.magnitude, ControlType.kSmartVelocity);
    }


    public synchronized void setBeltTransporter(boolean activated) {
        beltTransportMotor.set( activated ? Constants.kBeltTransporterVelocity : 0.0 );
    }

    public synchronized double getShooterVelocity() {
        return shooterMotor.getEncoder().getVelocity();
    }
    
    @Override
    public void init() {

        shooterMotor.setOpenLoopRampRate(1);
        shooterMotor.setSmartCurrentLimit(45);

        beltTransportMotor.setSmartCurrentLimit(45);

        shooterMotor.setIdleMode(IdleMode.kCoast);

        boolean setSucceeded;
        int retryCounter = 0;

        do {
            setSucceeded = true;

            setSucceeded &= shooterMotor.getEncoder().setMeasurementPeriod(10) == REVLibError.kOk;
            setSucceeded &= SparkHelper.setPIDGains(shooterMotor, 0, Constants.kShooterVelocityKp, Constants.kShooterVelocityKi, Constants.kShooterVelocityKd, Constants.kShooterVelocityKf, Constants.kShooterVelocityRampRate, 0);

            setSucceeded &= shooterMotor.getPIDController().setSmartMotionMaxVelocity(5300, 0) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getPIDController().setOutputRange(-1, 1) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0) == REVLibError.kOk;
            setSucceeded &= shooterMotor.getPIDController().setSmartMotionMaxAccel(5300, 0) == REVLibError.kOk;



            shooterMotor.selectProfileSlot(0, 0);
        } while(!setSucceeded && retryCounter++ < 3);

        if(!setSucceeded) {
            System.out.println("Shooter Initialization Failed");
        }
    }

    @Override
    public void subsystemHome() {
        shooterMotor.getEncoder().setPosition(0);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }
}
