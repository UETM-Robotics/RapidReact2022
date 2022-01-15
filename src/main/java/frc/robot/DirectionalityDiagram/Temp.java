package frc.robot.DirectionalityDiagram;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class Temp extends CommandBase {
    private final Timer timer = new Timer();
    private final boolean usePID;
    private final Trajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final RamseteController follower;
    private final SimpleMotorFeedforward feedforward;
    private final DifferentialDriveKinematics kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> speeds;
    private final PIDController leftController;
    private final PIDController rightController;
    private final BiConsumer<Double, Double> output;
    private DifferentialDriveWheelSpeeds prevSpeeds;
    private double prevTime;


    /**
     * @param Trajectory | Trajectory to follow
     * 
     * @param Supplier<Pose2d> | Function that supplies the robot pose
     * 
     * @param RamseteController | Trajectory following parser
     * 
     * @param DifferentialDriveKinematics | The drivetrain kinematics object
     * 
     * @param Supplier<DifferentialDriveWheelSpeeds> | A function that supplies the speeds of the left and right sides of the robot
     *   drive.
     * 
     * @param PIDController | Left Drive Side PID Controller
     * 
     * @param PIDController | Right Drive Side PID Controller
     * 
     * @param BiConsumer<Double, Double> | Function that sets the output voltage of the drive motors
     */
    public Temp(Trajectory trajectory,
        Supplier<Pose2d> pose,
        RamseteController controller,
        DifferentialDriveKinematics kinematics,
        Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
        PIDController leftController,
        PIDController rightController,
        BiConsumer<Double, Double> outputVolts) {


          this.feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);


          this.trajectory = trajectory;
          this.pose = pose;
          this.follower = controller;
          this.kinematics = kinematics;
          this.speeds = wheelSpeeds;
          this.leftController = leftController;
          this.rightController = rightController;
          this.output = outputVolts;


          
          this.usePID = true;
    }

    @Override
    public void initialize() {

        prevTime = -1;
        var initialState = trajectory.sample(0);
        prevSpeeds = kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
            )
        );

        timer.reset();
        timer.start();

        if(usePID) {
            leftController.reset();
            rightController.reset();
        }

    }

    @Override
    public void execute() {
        double curTime = timer.get();
        double dt = curTime - prevTime;

        if(prevTime < 0) {
            output.accept(0.0 , 0.0);
            prevTime = curTime;
            return;
        }

        var targetWheelSpeeds = kinematics.toWheelSpeeds(
            follower.calculate(pose.get(), trajectory.sample(curTime))
        );

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if(usePID) {
            double leftFeedForward = feedforward.calculate(
                leftSpeedSetpoint, 
                (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt
            );

            double rightFeedForward = feedforward.calculate(
                rightSpeedSetpoint, 
                (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt
            );

            leftOutput = 
                leftFeedForward + 
                leftController.calculate(speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

            rightOutput = 
                rightFeedForward + 
                rightController.calculate(speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        //output.accept(leftOutput, rightOutput);
        SmartDashboard.putNumber("left Output", leftOutput);
        SmartDashboard.putNumber("left Setpoint", leftSpeedSetpoint);
        SmartDashboard.putNumber("right Output", rightOutput);
        SmartDashboard.putNumber("right Setpoint", rightSpeedSetpoint);
        

        prevSpeeds = targetWheelSpeeds;
        prevTime = curTime;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        if(interrupted) {
            output.accept(0.0, 0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
  
}
