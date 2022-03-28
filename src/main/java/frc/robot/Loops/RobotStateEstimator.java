package frc.robot.Loops;

import frc.robot.RobotState;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.RamseteTrajectory.DifferentialDriveOdometry;
import frc.robot.subsystems.DriveTrain;

public class RobotStateEstimator implements Loop{

    private static final RobotStateEstimator instance = new RobotStateEstimator();

    DriveTrain drive_ = DriveTrain.getInstance();
    RobotState robotState = RobotState.getInstance();

    private DifferentialDriveOdometry odometry;


    public static RobotStateEstimator getInstance() {
        return instance;
    }

    private RobotStateEstimator() {
        odometry = new DifferentialDriveOdometry( drive_.getRotation() , new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }
    


    @Override
    public void onFirstStart(double timestamp) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onStart(double timestamp) {
        // TODO Auto-generated method stub
        odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
    }

    @Override
    public void onLoop(double timestamp, boolean isAuto) {
        synchronized (RobotStateEstimator.this) {

            robotState.updateFieldToRobotPose( odometry.update( drive_.getRotation(), 
            drive_.getLeftDistanceMeters(), drive_.getRightDistanceMeters() ) );

        }
    }

    @Override
    public void onStop(double timestamp) {
        // TODO Auto-generated method stub
        
    }

    public void resetOdometry() {
        drive_.subsystemHome();
        odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), drive_.getRotation());
    }

    public void resetOdometry(Pose2d startPose) {
        drive_.subsystemHome();
        odometry.resetPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), drive_.getRotation());
        robotState.setInitialPose(startPose);
    }

}
