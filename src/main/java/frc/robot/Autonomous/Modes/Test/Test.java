package frc.robot.Autonomous.Modes.Test;

import com.pathplanner.lib.PathPlanner;

import frc.robot.RobotState;
import frc.robot.Actions.DriveTrajectoryAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Loops.RobotStateEstimator;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.RamseteTrajectory.Trajectory;


public class Test extends AutoModeBase {

    Trajectory testPathStepOne = Trajectory.fromWpiLibTrajectory(PathPlanner.loadPath("TestPathStepOne", 4.0, 2.0));
    Trajectory testPathStepTwo = Trajectory.fromWpiLibTrajectory(PathPlanner.loadPath("TestPathStepTwo", 4.0, 2.0));

    @Override
    protected void routine() throws AutoModeEndedException {

        RobotStateEstimator.getInstance().resetOdometry( new Pose2d(9.18, 1.45, Rotation2d.fromDegrees(180)) );
        
        runAction(new DriveTrajectoryAction(testPathStepOne));

    }
    


}
