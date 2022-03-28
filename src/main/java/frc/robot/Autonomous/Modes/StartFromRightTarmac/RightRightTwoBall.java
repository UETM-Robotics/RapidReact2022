package frc.robot.Autonomous.Modes.StartFromRightTarmac;



import frc.robot.Actions.AimAtHub;
import frc.robot.Actions.DriveTrajectoryAction;
import frc.robot.Autonomous.Framework.AutoModeBase;
import frc.robot.Autonomous.Framework.AutoModeEndedException;
import frc.robot.Loops.RobotStateEstimator;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.PathPlanner.PathPlanner;
import frc.robot.Utilities.RamseteTrajectory.Trajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrain.DriveControlState;
import frc.robot.subsystems.Intake.IntakeControlMode;

public class RightRightTwoBall extends AutoModeBase {

    //Trajectory trajectory = Trajectory.fromWpiLibTrajectory(PathPlanner.loadPath("RightRightTwoBall", 4, 4));
    Trajectory trajectory = PathPlanner.loadPath("RightRightTwoBall", 4.0, 4.0);

    @Override
    protected void routine() throws AutoModeEndedException {

        RobotStateEstimator.getInstance().resetOdometry(new Pose2d(7.66, 1.96, Rotation2d.fromDegrees(-90)));

        DriveTrain.getInstance().setBrakeMode(true);
        DriveTrain.getInstance().setControlMode(DriveControlState.PATH_FOLLOWING);


        //Intake.getInstance().setControlMode(IntakeControlMode.ENABLED);
        
        runAction(new DriveTrajectoryAction(trajectory));

        //Intake.getInstance().setControlMode(IntakeControlMode.DISABLED);

        //runAction(new AimAtHub(true));

    }
    
}
