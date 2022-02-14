package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Actions.Autonomy.AutomatedAction;
import frc.robot.Actions.Framework.CrashTrackingRunnable;
import frc.robot.Actions.Framework.TeleopActionRunner;
import frc.robot.Actions.OperatedActions.SetIntakeAction;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;
import frc.robot.Utilities.TrajectoryFollowingMotion.DriveSignal;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Utilities.Constants;
import frc.robot.Utilities.ControlsConsumerU;
import frc.robot.Utilities.DriveControlState;
import frc.robot.Utilities.DriveMotorValues;

public class HIDController {
    
    private static HIDController instance = new HIDController();

    public synchronized static HIDController getInstance() {
        return instance;
    }

    private final ControllerU driverController = new ControllerU(0);

    private final Object taskRunningLock_ = new Object();

    private boolean firstRun = true;

    private static double mThrottle = 0;
	private static double mTurn = 0;
	private static final DriveSignal mDriveSignalOutput = new DriveSignal(0, 0);

    private static final double HID_RATE_CONTROL = 0.020;

    private final Notifier mHIDNotifier;

    private final ArrayList<BooleanSupplier> mControlFunctions = new ArrayList<>();


    DriveTrain dTrain = DriveTrain.getInstance();


    private final CrashTrackingRunnable mHIDRunnable = new CrashTrackingRunnable() {

        @Override
		public void runCrashTracked() {
			synchronized (taskRunningLock_) {
				if (firstRun) {
					Thread.currentThread().setName("DriveControls");
					Thread.currentThread().setPriority(Constants.kControllerThreadPriority);
					firstRun = false;
				}
				try {
					for (BooleanSupplier b : mControlFunctions) {
						b.getAsBoolean();   //TODO: Generate report of active functions
					}
				} catch (Exception ex) {
                    DriverStation.reportError("Fatal Error Running HID", true);
				} catch (Throwable t) {
                    DriverStation.reportError("Fatal Error Running HID", true);
				}
			}

			TeleopActionRunner.processActions();
		}
    };


    private HIDController() {
        mHIDNotifier = new Notifier(mHIDRunnable);
		registerControls();
    }

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

	private void registerButtonPressControl(ControllerU joystick, int button, ControlsConsumerU controlFunction) {
		mControlFunctions.add(() -> {
			if (joystick.getRisingEdgeButton(button)) {
				controlFunction.accept(joystick, button);
				return true;
			}
			return false;
		});
	}

    private void registerControls() {

		//DriveTrain drive Controls
        mControlFunctions.add(() -> {
			if (dTrain.getDriveControlState() == DriveControlState.OPEN_LOOP) {
				
				mThrottle = -driverController.getNormalizedAxis(1, Constants.kJoystickDeadband);
				mTurn = driverController.getNormalizedAxis(4, Constants.kJoystickDeadband);
				dTrain.setBrakeMode(driverController.getRawButton(5));
				mDriveSignalOutput.set(Math.max(Math.min(mThrottle + mTurn, 1), -1), Math.max(Math.min(mThrottle - mTurn, 1), -1));
				dTrain.setDriveOpenLoop( DriveMotorValues.fromDriveSignal(mDriveSignalOutput) );
				return true;
			} else if(dTrain.getDriveControlState() == DriveControlState.AUTO_AIMING) {
				mThrottle = 0;
				mTurn = 0;
				return true;
			}
			mThrottle = 0;
			mTurn = 0;
			return false;
		});

		//Intake On Button
		registerButtonPressControl(driverController, 1, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
					new SetIntakeAction(false, () -> j.getRawButton(b)), 300));
		});

		//Intake Out Button
		registerButtonPressControl(driverController, 3, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
					new SetIntakeAction(true, () -> j.getRawButton(b)), 300));
		});

    }

}
