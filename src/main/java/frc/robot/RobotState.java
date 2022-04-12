package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import frc.robot.Utilities.RobotRelativeSpeed;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Geometry.Translation2d;
import frc.robot.Utilities.TrajectoryFollowing.DifferentialDriveKinematics;
import frc.robot.Utilities.TrajectoryFollowing.DifferentialDriveWheelSpeeds;
import frc.robot.Utilities.TrajectoryFollowing.Trajectory.State;



public class RobotState {

    private static final RobotState instance = new RobotState();

    private DifferentialDriveKinematics kinematics;

    List<Translation2d> mCameraToVisionTargetPoses = new ArrayList<>();

    private Pose2d mCurrentPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private Pose2d initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

    private Translation2d mFieldToHub = new Translation2d(TechConstants.kHubPositionXMeters,
                                            TechConstants.kHubPositionYMeters);

    //TODO: DETERMINE VEHICLE TO SHOOTER POSE
    //private Pose2d vehicle_to_shooter_ = new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)); //shooter w.r.t. robot


    public static RobotState getInstance() {
        return instance;
    }

    private RobotState() {
        kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22.25));
    }


    public DifferentialDriveWheelSpeeds toWheelSpeeds(State state) {
        return kinematics.toWheelSpeeds(
            new RobotRelativeSpeed(
                state.velocityMetersPerSecond,
                0,
                state.curvatureRadPerMeter * state.velocityMetersPerSecond
            )
        );
    }

    public DifferentialDriveWheelSpeeds toWheelSpeeds(RobotRelativeSpeed robotSpeed) {
        return kinematics.toWheelSpeeds(robotSpeed);
    }


    public synchronized void updateFieldToRobotPose( Pose2d pose ) {

     mCurrentPose = pose.relativeTo(initialPose);

    //  mCurrentPose = new Pose2d(pose.x() + initialPose.x(), 
    //                             pose.y() + initialPose.y(), 
    //                             Rotation2d.fromDegrees(pose.getRotation().getDegrees() + initialPose.getRotation().getDegrees()));

        //mCurrentPose = pose;
    }

    public synchronized void updateFieldToHubPose(double distanceFromRobot) {

        mFieldToHub = mCurrentPose.getTranslation().plus( new Translation2d(distanceFromRobot * Math.cos(mCurrentPose.getRotation().getDegrees()),
                                                          distanceFromRobot * Math.sin(mCurrentPose.getRotation().getDegrees())) );

    }


    public synchronized void setInitialPose(Pose2d pose) {
        initialPose = new Pose2d(pose.x(), pose.y(), Rotation2d.fromDegrees(pose.getRotation().getDegrees()));
    }


    public synchronized Pose2d getFieldToVehicleMeters() {
        return mCurrentPose;
    }

    public synchronized Pose2d getFieldToVehicleInches() {
        return new Pose2d(Units.metersToInches(mCurrentPose.x()), Units.metersToInches(mCurrentPose.y()), mCurrentPose.getRotation());
    }

    public synchronized Translation2d getFieldToHubMeters() {
        return mFieldToHub;
    }

}