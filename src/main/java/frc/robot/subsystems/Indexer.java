package frc.robot.subsystems;

import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Loops.Loop;
import frc.robot.Utilities.Loops.Looper;

public class Indexer extends Subsystem implements CustomSubsystem {


    private static Indexer instance = new Indexer();
    private static ReentrantLock _subsystemMutex = new ReentrantLock();
    private Controllers controlers = Controllers.getInstance();

    private IndexerControlMode mControlMode = IndexerControlMode.DISABLED;
    
    public static Indexer getInstance() {
        return instance;
    }


    private SparkMaxU indexerLeft;
    private SparkMaxU indexerRight;
    

    private Indexer() {
        indexerLeft = controlers.getIndexerLeftMotor();
        indexerRight = controlers.getIndexerRightMotor();
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
            switch(mControlMode) {
                case DISABLED:
                    indexerLeft.set(0);
                    indexerRight.set(0);
                    break;
                case ENABLED:
                    indexerLeft.set(0.2);
                    indexerRight.set(0.2);
                    break;
                default:
                    indexerLeft.set(0);
                    indexerRight.set(0);
                    DriverStation.reportError("Exception in Indexer onLoop", false);
                    break;

            }
        }

        @Override
        public void onStop(double timestamp) {
            stop();
        }

    };

    public synchronized void set() {
        indexerLeft.set(0.2);
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


    @Override
    public void init() {
        indexerLeft.setOpenLoopRampRate(0.3);
        indexerLeft.setOpenLoopRampRate(0.3);

        indexerLeft.setSmartCurrentLimit(45);
        indexerRight.setSmartCurrentLimit(45);
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
        DISABLED;
    }
    
}
