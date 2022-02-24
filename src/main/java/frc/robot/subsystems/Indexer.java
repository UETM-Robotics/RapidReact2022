package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;

public class Indexer extends Subsystem implements CustomSubsystem {


    private static Indexer instance = new Indexer();
    private static ReentrantLock _subsystemMutex = new ReentrantLock();
    private Controllers controllers = Controllers.getInstance();

    private IndexerControlMode mControlMode = IndexerControlMode.DISABLED;
    private BeltControlMode mBeltControlMode = BeltControlMode.DISABLED;
    
    public static Indexer getInstance() {
        return instance;
    }


    private SparkMaxU indexerLeft;
    private SparkMaxU indexerRight;
    
    public final SparkMaxU beltTransporterMotor;
    

    private Indexer() {
        indexerLeft = controllers.getIndexerLeftMotor();
        indexerRight = controllers.getIndexerRightMotor();

        beltTransporterMotor = controllers.getBeltTransporterMotor();
    }


    private Loop mLoop = new Loop() {

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
            synchronized(Indexer.this) {
                switch(mControlMode) {
                    case DISABLED:
                        indexerLeft.set(0);
                        indexerRight.set(0);
                        break;
                    case ENABLED:
                        indexerLeft.set(-0.2);
                        indexerRight.set(0.2);
                        break;
                    case UNJAM:
                        indexerLeft.set(0.22);
                        indexerRight.set(0.22);
                    default:
                        indexerLeft.set(0);
                        indexerRight.set(0);
                        DriverStation.reportError("Anomaly in Indexer onLoop", false);
                        break;
    
                }

                switch(mBeltControlMode) {
                    case DISABLED:
                        beltTransporterMotor.set(0);
                        break;
                    case ENABLED:
                        beltTransporterMotor.set(0.6);
                        break;
                    case REVERSE:
                        beltTransporterMotor.set(-0.2);
                        break;
                    default:
                        beltTransporterMotor.set(0);
                        DriverStation.reportError("Anomaly in Belt onLoop", true);
                        break;

                }
            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }

    };

    public synchronized void set() {
        indexerLeft.set(-0.2);
        indexerRight.set(0.2);
    }


    public synchronized void setIndexerControlMode( IndexerControlMode controlMode ) {

        if (controlMode != mControlMode) {
			try {
				_subsystemMutex.lock();
				mControlMode = controlMode;
				_subsystemMutex.unlock();
			} catch (Exception ex) {
                DriverStation.reportError("Failure to set Indexer Control Mode", true);
			}
		}

    }

    public synchronized void setBeltControlMode( BeltControlMode controlMode ) {

        if(controlMode != mBeltControlMode) {
            try {
                _subsystemMutex.lock();
                mBeltControlMode = controlMode;
                _subsystemMutex.unlock();
            } catch( Exception ex ) {
                DriverStation.reportError("Failure to set Belt Control Mode", true);
            }
        }

    }


    @Override
    public void init() {
        indexerLeft.setOpenLoopRampRate(0.3);
        indexerLeft.setOpenLoopRampRate(0.3);

        indexerLeft.setSmartCurrentLimit(45);
        indexerRight.setSmartCurrentLimit(45);

        indexerLeft.setIdleMode(IdleMode.kCoast);
        indexerRight.setIdleMode(IdleMode.kCoast);

        beltTransporterMotor.setIdleMode(IdleMode.kCoast);
        beltTransporterMotor.setOpenLoopRampRate(0.5);
        beltTransporterMotor.setSmartCurrentLimit(40);
        beltTransporterMotor.getEncoder().setMeasurementPeriod(10);
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

    public enum IndexerControlMode {
        ENABLED,
        UNJAM,
        DISABLED;
    }

    public enum BeltControlMode {
        ENABLED,
        REVERSE,
        DISABLED;
    }
    
}
