package frc.robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Actions.AimAtHub;
import frc.robot.Actions.SetShooterAction;
import frc.robot.Actions.Autonomy.AutomatedAction;
import frc.robot.Actions.Framework.TeleopActionRunner;
import frc.robot.Actions.OperatedActions.SetBeltAction;
import frc.robot.Actions.OperatedActions.SetIntakeAction;
import frc.robot.Loops.CrashTrackingRunnable;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.ControlsConsumerU;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;
import frc.robot.Utilities.Drivers.rcinput.ControllerU.Direction;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.DriveTrain.DriveControlState;

public class HIDController {
    
    private static final HIDController instance = new HIDController();

    public synchronized static HIDController getInstance() {
        return instance;
    }

    private final Controllers controllers = Controllers.getInstance();

    private final ControllerU driverController;
    private final ControllerU operatorController;

    private final Object taskRunningLock_ = new Object();

    private boolean firstRun = true;


    private static final double HID_RATE_CONTROL = 0.020;

    private final Notifier mHIDNotifier;

    private final ArrayList<BooleanSupplier> mControlFunctions = new ArrayList<>();

    private final CrashTrackingRunnable mHIDRunnable = new CrashTrackingRunnable() {

        @Override
		public void runCrashTracked() {
			synchronized (taskRunningLock_) {
				if (firstRun) {
					Thread.currentThread().setName("DriveControls");
					Thread.currentThread().setPriority(TechConstants.kControllerThreadPriority);
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
		driverController = controllers.getDriverController();
		operatorController = controllers.getOperatorController();
		registerControls();
    }

    public void start() {
		synchronized (taskRunningLock_) {
			mHIDNotifier.startPeriodic(HID_RATE_CONTROL);
			//Pneumatics.getInstance().setCompressor(true);
		}
	}

	public void stop() {
		synchronized (taskRunningLock_) {
			mHIDNotifier.stop();
			DriveTrain.getInstance().setControlMode(DriveControlState.DISABLED);
		}
	}

	/**
     * Register a button press control: Face Buttons, Bumpers, Start, and Share Buttons
     *
     * @param joystick the target joystick
     * @param button the button that will activate the command
     * @param controlFunction the lambda function that will run the command
     */
	private void registerButtonPressControl( ControllerU joystick, int button, ControlsConsumerU controlFunction ) {
		mControlFunctions.add(() -> {
			if (joystick.getRisingEdgeButton(button)) {
				controlFunction.accept(joystick, button);
				return true;
			}
			return false;
		});
	}

	/**
     * Register a trigger press control
     *
     * @param joystick the target joystick
     * @param trigger the trigger that will activate the command
     * @param controlFunction the lambda function that will run the command
     */
	private void registerTriggerPressControl( ControllerU joystick, int trigger, ControlsConsumerU controlFunction ) {
		mControlFunctions.add( () -> {
			if( joystick.getTriggerPressed(trigger, TechConstants.kControllerTriggerThreshold) ) {
				controlFunction.accept( joystick, trigger );

				return true;
			}

			return false;
		});
	}

	private void registerDpadControl( ControllerU joystick, Direction POV, ControlsConsumerU controlFunction ) {
		mControlFunctions.add( () -> {
			if(joystick.getRisingEdgeDpad(POV)) {
				controlFunction.accept( joystick, 0);

				return true;
			}

			return false;
		} );
	}

	/**
     * Register a DpadUp press control
     *
     * @param joystick the target joystick
     * @param controlFunction the lambda function that will run the command
     */
	private void registerDpadUpControl(ControllerU joystick, ControlsConsumerU controlFunction) {
		mControlFunctions.add( () -> {
			if(joystick.getRisingUpDpad()) {
				controlFunction.accept(joystick, 0);
				return true;
			}
			return false;
		} );
	}

		/**
     * Register a DpadDown press control
     *
     * @param joystick the target joystick
     * @param controlFunction the lambda function that will run the command
     */
	private void registerDpadDownControl(ControllerU joystick, ControlsConsumerU controlFunction) {
		mControlFunctions.add( () -> {
			if(joystick.getRisingDownDpad()) {
				controlFunction.accept(joystick, 0);
				return true;
			}
			return false;
		} );
	}


    private void registerControls() {

		// registerButtonPressControl(driverController, 1, (j, b) -> {
		// 	DriveTrain.getInstance().subsystemHome();
		// 	RobotStateEstimator.getInstance().resetOdometry();
		// });

		//Intake On
		registerButtonPressControl(driverController, 7, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new SetIntakeAction(false, () -> j.getRawButton(b)), 300));
		});

		//Intake Out Button
		registerButtonPressControl(driverController, 1, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new SetIntakeAction(true, () -> j.getRawButton(b)), 300));
		});

		//Driver toggle compressor
		registerButtonPressControl(driverController, 4, (j, b) -> {
			Pneumatics.getInstance().setCompressor(!Pneumatics.getInstance().getCompressor());
		});


		//Auto Aim
		registerButtonPressControl(driverController, 3, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new AimAtHub(false, () -> j.getRawButton(b)), 300));
		});


		registerTriggerPressControl(driverController, 4, (j, b) -> {
			TeleopActionRunner.runAction(AutomatedAction.fromAction(
				new SetShooterAction(false, () -> j.getTriggerPressed(b, TechConstants.kControllerTriggerThreshold)), 300));
		});

		// registerButtonPressControl(driverController, 6, (j, b) -> {
		// 	TeleopActionRunner.runAction(AutomatedAction.fromAction(
		// 		new SetBeltAction(false, () -> j.getRawButton(b)), 300));
		// });

		// registerButtonPressControl(driverController, 5, (j, b) -> {
		// 	TeleopActionRunner.runAction(AutomatedAction.fromAction(
		// 		new SetBeltAction(true, () -> j.getRawButton(b)), 300));
		// });


		//Logger
		mControlFunctions.add( () -> {

			// SmartDashboard.putNumber("Left Distance Driven", DriveTrain.getInstance().getLeftDistanceMeters());
			// SmartDashboard.putNumber("Right Distance Driven", DriveTrain.getInstance().getRightDistanceMeters());
			// SmartDashboard.putNumber("DriveTrain Rotation", DriveTrain.getInstance().getRotation().getDegrees());


			
			SmartDashboard.putNumber("Robots guessed angle", DriveTrain.getInstance().getHubAimingGuess());

			return true;
		});

    }

}
