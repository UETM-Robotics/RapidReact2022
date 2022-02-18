package frc.robot.Utilities.Loops;

import frc.robot.subsystems.DriveTrain;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Geometry.Twist2d;
import frc.robot.Utilities.TrajectoryFollowingMotion.Kinematics;
import frc.robot.Utilities.TrajectoryFollowingMotion.PathFollowerRobotState;

/**
 * Periodically estimates the state of the robot using the robot's distance traveled (compares two waypoints), gyroscope
 * orientation, and velocity, among various other factors. Similar to a car's odometer.
 */
public class RobotStateEstimator implements Loop {
    static RobotStateEstimator instance_ = new RobotStateEstimator();

    public static RobotStateEstimator getInstance() {
        return instance_;
    }

    RobotStateEstimator() {
    }

    PathFollowerRobotState robot_state_ = PathFollowerRobotState.getInstance();
    DriveTrain drive_ = DriveTrain.getInstance();
    double left_encoder_prev_distance_ = 0;
    double right_encoder_prev_distance_ = 0;
    double gyroscope_prev_angle = 0;

    @Override
    public void onFirstStart(double timestamp) {

    }

    @Override
    public synchronized void onStart(double timestamp) {
        left_encoder_prev_distance_ = drive_.getLeftDistanceInches();
        right_encoder_prev_distance_ = drive_.getRightDistanceInches();
        gyroscope_prev_angle = drive_.getGyroAngle().getDegrees();
    }

    @Override
    public synchronized void onLoop(double timestamp, boolean isAuto) {
        final double left_distance = drive_.getLeftDistanceInches();
        final double right_distance = drive_.getRightDistanceInches();
        //final Rotation2d gyro_angle = Rotation2d.fromDegrees(0);
        //final Rotation2d gyro_angle = drive_.getGyroAngle();
        final Rotation2d gyro_angle = Rotation2d.fromDegrees( drive_.getGyroAngle().getDegrees() - gyroscope_prev_angle );

        final Twist2d odometry_velocity = robot_state_.generateOdometryFromSensors(
                left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(drive_.getLeftVelocityInchesPerSec(),
                drive_.getRightVelocityInchesPerSec());
        robot_state_.addObservations(timestamp, odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

    @Override
    public void onStop(double timestamp) {
        // no-op
    }

}
