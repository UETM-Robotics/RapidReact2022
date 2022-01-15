// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.nio.file.Path;

import com.fasterxml.jackson.databind.ser.std.RawSerializer;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.DirectionalityDiagram.KinematicDriveSchematic;
import frc.robot.DirectionalityDiagram.TraversalDriveSchematic;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FlyWheel;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private DriveTrain dTrain;

  private XboxController ctrl = new XboxController(0);

  private TraversalDriveSchematic auto;
  private RamseteController controller;

  private FlyWheel flywheel = new FlyWheel();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    dTrain = DriveTrain.getInstance();
    
    controller = new RamseteController();

    dTrain.resetEncoder();
    dTrain.resetGyro();
    dTrain.resetOdometry();
    dTrain.resetPose();

    dTrain.xEntry.setNumber(0);
    dTrain.yEntry.setNumber(0);
    dTrain.theta.setNumber(0);

    
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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    dTrain.resetEncoder();
    dTrain.resetGyro();
    dTrain.resetOdometry();
    dTrain.resetPose();

    dTrain.setBrake();

    auto = new TraversalDriveSchematic("paths/Test.wpilib.json", dTrain::getPose, controller, dTrain.kinematics, dTrain::getSpeeds, dTrain.leftController, dTrain.rightController, dTrain::setOutput);
  
    //autoA = new RamseteCommand(trajectoryA, dTrain::getPose, controller, dTrain.kinematics, dTrain::outputMPS, dTrain);
    //autoB = new RamseteCommand(trajectoryB, dTrain::getPose, controller, dTrain.kinematics, dTrain::outputMPS, dTrain);
    //auto = new RamseteCommand(trajectory, dTrain::getPose, controller, dTrain.feedforward, dTrain.kinematics, dTrain::getSpeeds, dTrain.leftController, dTrain.righController, dTrain::setOutput, dTrain);
    auto.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if(auto.isFinished()) {
      dTrain.driveTank(0, 0);
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    CommandScheduler.getInstance().cancelAll();

    // dTrain.resetEncoder();
    // dTrain.resetGyro();
    // dTrain.resetOdometry();
    // dTrain.resetPose();

    // dTrain.xEntry.setNumber(0);
    // dTrain.yEntry.setNumber(0);
    // dTrain.theta.setNumber(0);

    dTrain.setCoast();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    dTrain.driveTank(-(ctrl.getRawAxis(1) - ctrl.getRawAxis(4)), -( ctrl.getRawAxis(1) + ctrl.getRawAxis(4)));
    SmartDashboard.putNumber("degrees", dTrain.getHeading().getRadians());

    flywheel.Set(0.1);

    SmartDashboard.putNumber("percent", -flywheel.get());
    SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());

    //SmartDashboard.putNumber("temperature", );
    SmartDashboard.putNumber("temperature", flywheel.getTemp());

    // dTrain.driveTank(1, 1);

    // SmartDashboard.putNumber("left velocity mps", dTrain.getSpeeds().leftMetersPerSecond);
    // SmartDashboard.putNumber("right velocity mps", dTrain.getSpeeds().rightMetersPerSecond);
  }
  
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //auto.cancel();

    dTrain.resetEncoder();
  }

  /** This function is called periodically during test mode. */


  @Override
  public void testPeriodic() {
    // //dTrain.driveTank(-0.1, -0.1);
    


  }
}
