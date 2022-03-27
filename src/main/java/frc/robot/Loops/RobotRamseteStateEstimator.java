package frc.robot.Loops;

import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.RamseteTrajectory.DifferentialDriveKinematics;
import frc.robot.Utilities.RamseteTrajectory.DifferentialDriveOdometry;

public class RobotRamseteStateEstimator implements Loop {


    private static final RobotRamseteStateEstimator instance = new RobotRamseteStateEstimator();

    public static RobotRamseteStateEstimator getInstance() {
        return instance;
    }

    private RobotRamseteStateEstimator() {}


    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(22.5);
    DifferentialDriveOdometry odometry;


    @Override
    public void onFirstStart(double timestamp) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onStart(double timestamp) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onLoop(double timestamp, boolean isAuto) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void onStop(double timestamp) {
        // TODO Auto-generated method stub
        
    }
    

    public void reset() {
        odometry = new DifferentialDriveOdometry( Rotation2d.fromDegrees(0) );
    }

}
