package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.Drivers.SparkMaxU;

public class Intake extends Subsystem implements CustomSubsystem {

    private static final Intake instance = new Intake();
    private static ReentrantLock _subsystemMutex = new ReentrantLock();

    private final SparkMaxU intakeMotor;
    private final Solenoid intakeSolenoid;

    private IntakeControlMode mControlMode = IntakeControlMode.DISABLED;


    public static Intake getInstance() {
        return instance;
    }

    private Intake() {
        intakeMotor = Controllers.getInstance().getIntakeMotor();
        intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    }


    private final Loop mLoop = new Loop() {

        @Override
        public void onFirstStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onStart(double timestamp) {
            // TODO Auto-generated method stub
            
        }

        @Override
        public void onLoop(double timestamp, boolean isAuto) {
            synchronized (Intake.this) {

                switch(mControlMode) {

                    case DISABLED: 
                        intakeMotor.set(0);
                        break;
                    case ENABLED:
                        intakeMotor.set(-0.8);
                        break;
                    case REVERSE:
                        intakeMotor.set(0.9);
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
            intakeSolenoid.set(false);

            intakeMotor.set(0);
        }

    };


    public void setControlMode(IntakeControlMode controlMode) {

		if (controlMode != mControlMode) {

			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                
			}

            if(controlMode != IntakeControlMode.DISABLED)
                intakeSolenoid.set(true);
            else
                intakeSolenoid.set(false);
		}

	}

    public void prepareToEject() {
        intakeMotor.setSmartCurrentLimit(80);
        intakeMotor.setOpenLoopRampRate(0.2);
    }

    public void revert() {
        intakeMotor.setSmartCurrentLimit(50);
        intakeMotor.setOpenLoopRampRate(1);
    }


    @Override
    public void init() {
        intakeMotor.setSmartCurrentLimit(50);
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setOpenLoopRampRate(1);
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
    

    public enum IntakeControlMode {
        ENABLED,
        REVERSE,
        DISABLED;
    }
}
