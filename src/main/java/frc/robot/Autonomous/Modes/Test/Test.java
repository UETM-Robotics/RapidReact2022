package frc.robot.Autonomous.Modes.Test;


import frc.robot.RobotState;
import frc.robot.Actions.DriveTrajectoryAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Loops.RobotStateEstimator;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.PathPlanner.PathPlanner;
import frc.robot.Utilities.RamseteTrajectory.Trajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;


public class Test extends AutoModeBase {

    //Trajectory testPathStepOne = Trajectory.fromWpiLibTrajectory(PathPlanner.loadPath("TestPathStepOne", 1.0, 2.0));
    //Trajectory testPathStepTwo = Trajectory.fromWpiLibTrajectory(PathPlanner.loadPath("TestPathStepTwo", 4.0, 2.0));

    Trajectory testPathStepOne = PathPlanner.loadPath("TestPathStepOne", 2.0, 1.0);
    Trajectory testPathStepTwo = PathPlanner.loadPath("TestPathStepTwo", 2.0, 1.0);
    
    @Override
    protected void routine() throws AutoModeEndedException {

        Pneumatics.getInstance().setCompressor(false);

        DriveTrain.getInstance().subsystemHome();
        RobotStateEstimator.getInstance().resetOdometry( new Pose2d(9.78, 1.45, Rotation2d.fromDegrees(180)) );
        
        runAction(new DriveTrajectoryAction(testPathStepOne, testPathStepOne.sample(0).poseMeters) );

        runAction(new DriveTrajectoryAction(testPathStepTwo, testPathStepTwo.sample(0).poseMeters));
    }
    


}
