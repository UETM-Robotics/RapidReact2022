// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeExecuter;
import frc.robot.Autonomous.Modes.BasicMode;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.DriveControlState;
import frc.robot.Utilities.DriveMotorValues;
import frc.robot.Utilities.ThreadRateControl;
import frc.robot.Utilities.Loops.Looper;
import frc.robot.Utilities.Loops.RobotStateEstimator;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private Controllers robotControllers;

  private Looper mLooper;
  
  private DriveTrain dTrain;
  private Intake intake;
  private Shooter shooter;

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

    robotControllers = Controllers.getInstance();

    mLooper = new Looper();

    dTrain = DriveTrain.getInstance();
    intake = Intake.getInstance();
    shooter = Shooter.getInstance();

    dTrain.init();
    dTrain.registerEnabledLoops(mLooper);

    intake.init();
    intake.registerEnabledLoops(mLooper);
    
    shooter.init();
    shooter.registerEnabledLoops(mLooper);

    mHidController = HIDController.getInstance();

    robotStateEstimator = RobotStateEstimator.getInstance();
    mLooper.register(robotStateEstimator);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    
    exitAuto();

    mLooper.stop();

    //shooter.setBeltTransporter(0.0);
    //shooter.setShooterVelocityChango(0.0);

    mHidController.stop();
    
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }


    mLooper.start(true);
		dTrain.setBrakeMode(true);
    dTrain.subsystemHome();
		autoModeExecuter = new AutoModeExecuter();

		AutoModeBase autoMode = new BasicMode();


		if (autoMode != null)
			autoModeExecuter.setAutoMode(autoMode);
		else
			return;

		autoModeExecuter.start();
		threadRateControl.start(true);

		while (isAutonomous() && isEnabled()) {
      
      threadRateControl.doRateControl(100);
    }

  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("gyro", dTrain.getGyroAngle().getDegrees());
    // dTrain.setBruh();
    // SmartDashboard.putNumber("left position", dTrain.leftFront.getEncoder().getPosition());
    // SmartDashboard.putNumber("right position", dTrain.rightFront.getEncoder().getPosition());

    // SmartDashboard.putNumber("left speed", dTrain.leftFront.getEncoder().getVelocity());
    // SmartDashboard.putNumber("right speed", dTrain.rightFront.getEncoder().getVelocity());
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.p
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    try {
      System.out.println("Beginning TeleOP");

			exitAuto();

      dTrain.setControlMode(DriveControlState.OPEN_LOOP);

			mLooper.start(true);

			dTrain.setDriveVelocity(DriveMotorValues.NEUTRAL, true);
			dTrain.setDriveOpenLoop(DriveMotorValues.NEUTRAL);
			dTrain.setBrakeMode(false);
			mHidController.start();

      dTrain.setControlMode(DriveControlState.OPEN_LOOP);

		} catch (Throwable t) {
      DriverStation.reportError("Fatal Error Initializing Teleop", true);
			throw t;
		}
    
    CommandScheduler.getInstance().cancelAll();
  }
  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }

  /** This function is called periodically during test mode. */

  

  @Override
  public void testPeriodic() {
    shooter.setShooterVelocityChango(10000);
    shooter.beltTransporterMotor.set(0.7);
    SmartDashboard.putNumber("Shooter Velocity", shooter.getvel());
  }



  private void exitAuto() {
		try {
			if (autoModeExecuter != null)
				autoModeExecuter.stop();


			autoModeExecuter = null;
		} catch (Throwable t) {
		}
	}
}
