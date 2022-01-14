package frc.robot.DirectionalityDiagram;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

public class TraversalDriveSchematic extends KinematicDriveSchematic {

    private final Timer timer = new Timer();
    private final boolean usePID;
    private Trajectory trajectory;
    private final Supplier<Pose2d> pose;
    private final RamseteController follower;
    private final DifferentialDriveKinematics  kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> speeds;
    private final BiConsumer<Double, Double> output;
    private DifferentialDriveWheelSpeeds prevSpeeds;
    private double prevTime;
    private final DriveTrain dTrain;

    private final double dirMod;

    private final PIDController pid = new PIDController(1.5511, 0.0, 0.0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1476, 2.8719, 0.26283);

    public TraversalDriveSchematic(
        String trajectoryPath,
        Supplier<Pose2d> pose,
        RamseteController follower,
        DifferentialDriveKinematics kinematics,
        Supplier<DifferentialDriveWheelSpeeds> speeds,
        BiConsumer<Double, Double> output,
        boolean reversed
    ) {

        super();

        usePID = false;

        try{
            Path trajPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryPath);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajPath);
        } catch(IOException ex) {
            trajectory = null;
            DriverStation.reportError("Unable to open trajectory: " + trajectoryPath, ex.getStackTrace());
        }

        //this.trajectory = trajectory;
        this.pose  = pose;
        this.follower = follower;
        this.kinematics = kinematics;
        this.speeds = speeds;
        this.output = output;

        if(reversed) {
            dirMod = -1.0;
        } else {
            dirMod = 1.0;
        }

        dTrain = DriveTrain.getInstance();

    }

    @Override
    public void initialize() {
        prevTime = -1;
        var initialState = trajectory.sample(0);
        prevSpeeds = kinematics.toWheelSpeeds
        (
            new ChassisSpeeds(
                initialState.velocityMetersPerSecond,
                0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
            )
        );

        

        SmartDashboard.putNumber("Path Duration", trajectory.getTotalTimeSeconds());

        dTrain.setBrake();

        dTrain.resetEncoder();
        dTrain.resetEncoder();
        

        timer.reset();
        timer.start();
        
    }


    @Override
    public void execute() {
        double curTime = timer.get();

        SmartDashboard.putNumber("currentTime", curTime);

        double dt = curTime - prevTime;

        if(prevTime < 0) {
            output.accept(0.0, 0.0);
            prevTime = curTime;
            return;
        }

        var targetWheelSpeeds = kinematics.toWheelSpeeds(
            follower.calculate(pose.get(), trajectory.sample(curTime))
        );

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond * dirMod;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond * dirMod;

        double leftOutput;
        double rightOutput;

        if(usePID) {
            double leftFeedForward = feedforward.calculate(
                leftSpeedSetpoint, leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond / dt
            );

            double rightFeedForward = feedforward.calculate(
                rightSpeedSetpoint, rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond / dt
            );

            leftOutput = leftFeedForward + pid.calculate(speeds.get().leftMetersPerSecond, leftSpeedSetpoint);
            rightOutput = rightFeedForward + pid.calculate(speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        output.accept(leftOutput, rightOutput);
        prevSpeeds = targetWheelSpeeds;
        prevTime = curTime;

        SmartDashboard.putNumber("left speed", leftSpeedSetpoint);
        SmartDashboard.putNumber("right speed", rightSpeedSetpoint);
        
        SmartDashboard.putNumber("left position", dTrain.getLeftPosition());
        SmartDashboard.putNumber("right position", dTrain.getRightPosition());

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
