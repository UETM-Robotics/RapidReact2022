package frc.robot;

import frc.robot.Utilities.Constants;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.ControlsConsumerU;
import frc.robot.Utilities.DriveControlState;
import frc.robot.Utilities.DriveMotorValues;
import frc.robot.Utilities.QuickMaths;
import frc.robot.Utilities.Drivers.ControllerU;
import frc.robot.Utilities.TrajectoryFollowingMotion.DriveSignal;
import frc.robot.Utilities.TrajectoryFollowingMotion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Actions.Framework.CrashTrackingRunnable;
import frc.robot.Constants.DriverConstants;;

public class HIDController implements Runnable {

    private static HIDController instance = null;
	private final Object taskRunningLock_ = new Object();

    private DriveTrain dTrain;

    private Controllers robotControllers;

    private ControllerU throttleJoystick;

    private final ArrayList<BooleanSupplier> mControlFunctions = new ArrayList<>();

    private static final DriveSignal mDriveSignal = new DriveSignal(0.0, 0.0);

    private final Notifier mHIDNotifier;
    private static final double HID_RATE_CONTROL = 0.020;

    private boolean isFirstRun = true;

    //DRIVE CONTROL VARS
    private double mThrottle;
    private double mTurn;


    private HIDController() throws Exception {

        robotControllers = Controllers.getInstance();

        mHIDNotifier = new Notifier(mHIDRunnable);

        dTrain = DriveTrain.getInstance();

        throttleJoystick = robotControllers.getThrottleJoystick();

        registerControls();

    }

    public static HIDController getInstance() {
        if(instance == null) {
            
            try {
                instance = new HIDController();
            } catch(Exception ex) {
                System.out.println("HID Instanstiation Error");
            }

            return instance;

        }
            
        return instance;
    }

    private final CrashTrackingRunnable mHIDRunnable = new CrashTrackingRunnable() {

        @Override
        public void runCrashTracked() {
            synchronized(taskRunningLock_) {
                if(isFirstRun) {
                    Thread.currentThread().setName("DriveControls");
					Thread.currentThread().setPriority(Constants.kLooperThreadPriority);
					isFirstRun = false;
                }

                try {
                    for (BooleanSupplier b : mControlFunctions) {
						b.getAsBoolean();   //TODO: Generate report of active functions
					}
                } catch (Exception ex) {
                    System.out.println("Failed Running HID Controls");
				} catch (Throwable t) {
                    System.out.println("Wow things are horribly wrong with the HID Controls");
				}

            }
        }
        
    };

    public void start() {
		synchronized (taskRunningLock_) {
			mHIDNotifier.startPeriodic(HID_RATE_CONTROL);
		}
	}

	public void stop() {
		synchronized (taskRunningLock_) {
			mHIDNotifier.stop();
		}
	}

    @Override
    public void run() {

        mHIDRunnable.runCrashTracked();
        
    }
    
    private void registerButtonPressControl(ControllerU joystick, int button, ControlsConsumerU controlFunctions) {
        
        mControlFunctions.add( () -> {
            
            if(joystick.getRisingEdgeButton(button)) {
                controlFunctions.accept(joystick, button);
                return true;
            }

            return false;

        } );

    }

    private void registerControls() {

        mControlFunctions.add( () -> {

            if(dTrain.getControlMode() == DriveControlState.OPEN_LOOP) {

                mThrottle = -throttleJoystick.getNormalizedAxis(1, Constants.kJoystickDeadband);
                mTurn = throttleJoystick.getNormalizedAxis(4, Constants.kJoystickDeadband);

                SmartDashboard.putNumber("throttle", mThrottle);
                SmartDashboard.putNumber("turn", mTurn);

                dTrain.setBrakeMode(throttleJoystick.getRawButton(DriverConstants.DRIVE_HOLD_BRAKE) || Vision.getInstance().isVisionEnabled());

                mDriveSignal.set(Math.max(Math.min(mThrottle + mTurn, 1), -1), Math.max(Math.min(mThrottle - mTurn, 1), -1));

                dTrain.setDriveOpenLoop( DriveMotorValues.fromDriveSignal(mDriveSignal) );

                return true;
            }

            return false;
        });

    } 
}
