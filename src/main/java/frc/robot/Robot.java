// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeExecuter;
import frc.robot.Autonomous.Modes.Test.Test;
import frc.robot.Loops.Looper;
import frc.robot.Loops.RobotStateEstimator;
import frc.robot.Utilities.ThreadRateControl;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.DriveTrain.DriveControlState;
import frc.robot.subsystems.Vision.LedMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  
  private Looper mLooper;
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
  
  private DriveTrain dTrain;
  private Vision vision;
  private Intake intake;
  private RobotStateEstimator robotStateEstimator;

  private AutoModeExecuter autoModeExecuter;
  private HIDController mHidController;

  private ThreadRateControl threadRateControl = new ThreadRateControl();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


    mLooper = new Looper();

    dTrain = DriveTrain.getInstance();
    vision = Vision.getInstance();
    intake = Intake.getInstance();

    dTrain.init();
    dTrain.registerEnabledLoops(mLooper);

    vision.init();
    vision.registerEnabledLoops(mLooper);

    intake.init();
    intake.registerEnabledLoops(mLooper);


    mHidController = HIDController.getInstance();

    robotStateEstimator = RobotStateEstimator.getInstance();
    mLooper.register(robotStateEstimator);

    mAutoModeSelector.updateModeCreator();

    vision.setLed(LedMode.OFF);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    mAutoModeSelector.outputToSmartDashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    exitAuto();

    mLooper.stop();

    mAutoModeSelector.reset();
    mAutoModeSelector.updateModeCreator();
    autoModeExecuter = new AutoModeExecuter();

    mHidController.stop();

  }

  @Override
  public void disabledPeriodic() {
    mAutoModeSelector.updateModeCreator();
    Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();

    if (autoMode.isPresent() && autoMode.get() != autoModeExecuter.getAutoMode()) {
      System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
      autoModeExecuter.setAutoMode(autoMode.get());
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    mLooper.start(true);
		dTrain.setBrakeMode(true);
    dTrain.subsystemHome();
    //pneumatics.setCompressor(true);

    autoModeExecuter.setAutoMode(new Test());

		autoModeExecuter.start();
		threadRateControl.start(true);

		while (isAutonomous() && isEnabled()) {
      
      threadRateControl.doRateControl(100);
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


    try {
      System.out.println("Beginning TeleOP");

			exitAuto();

      dTrain.setControlMode(DriveControlState.OPEN_LOOP);

      
			// dTrain.setDriveVelocity(DriveMotorValues.NEUTRAL, true);
			// dTrain.setDriveOpenLoop(DriveMotorValues.NEUTRAL);
			dTrain.setBrakeMode(false);
      

      //TODO: ONLY FOR DEBUGGING
      robotStateEstimator.resetOdometry( new Pose2d(0, 0, Rotation2d.fromDegrees(-90)) );

      
			mHidController.start();

      dTrain.setControlMode(DriveControlState.DRIVER_CONTROL);
      // shooter.setShooterControlMode(ShooterControlMode.DISABLED);
      // shooter.setShooterVelocity(-3100);
      // shooter.setTopRollerVelocity(-3100);

      // indexer.setIndexerControlMode(IndexerControlMode.DISABLED);

      // pneumatics.setCompressor(true);

      mLooper.start(true);


		} catch (Throwable t) {
      DriverStation.reportError("Fatal Error Initializing Teleop", true);
			throw t;
		}

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  private void exitAuto() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();


			autoModeExecuter = null;
		} catch (Throwable t) {
		}
	}
}
