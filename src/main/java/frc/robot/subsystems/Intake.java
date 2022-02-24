package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;

public class Intake extends Subsystem implements CustomSubsystem {

    private static Intake instance = new Intake();
    private Controllers controllers = Controllers.getInstance();

    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private ControlMode mControlMode = ControlMode.DISABLED;

    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            synchronized(Intake.this) {

            }
        }

        @Override
        public void onStart(double timestamp) {
            synchronized(Intake.this) {
                setControlMode(ControlMode.DISABLED);
                intakeMotor.set(0);
            }
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (Intake.this) {

                switch (mControlMode) {
                    case DISABLED: 
                        intakeMotor.set(0);
                        break;
                    case ENABLED:
                        intakeMotor.set(0.65);
                        break;
                    case REVERSE:
                        intakeMotor.set(-0.9);
                        break;
                    default:
                        intakeMotor.set(0);
                        DriverStation.reportError("Failed to set Intake Speed", false);
                        break;    
				}
            }
        }

        @Override
        public void onStop(double timestamp) {
            synchronized(Intake.this) {
                setControlMode(ControlMode.DISABLED);
                intakeMotor.set(0);
            }
        }
    };


    public static Intake getInstance() {
        return instance;
    }

    private final SparkMaxU intakeMotor;
    
    private Intake() {
        intakeMotor = controllers.getIntakeMotor();
    }

    public void prepareToEject() {
        intakeMotor.setSmartCurrentLimit(80);
        intakeMotor.setOpenLoopRampRate(0.2);
    }

    public void revert() {
        intakeMotor.setSmartCurrentLimit(60);
        intakeMotor.setOpenLoopRampRate(0.6);
    }

    public void setControlMode(ControlMode controlMode) {
		if (controlMode != mControlMode) {
			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}
		}
	}

    public ControlMode getControlMode() {
        return mControlMode;
    }

    public void set() {
        intakeMotor.set(0.45);
    }


    @Override
    public void init() {
        intakeMotor.setSmartCurrentLimit(60);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setOpenLoopRampRate(0.6);
    }

    @Override
    public void subsystemHome() {
        intakeMotor.getEncoder().setPosition(0);
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        in.register(mLoop);
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }
    

    public enum ControlMode {
        ENABLED,
        REVERSE,
        DISABLED;
    }
}
