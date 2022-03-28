package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.util.Units;
import frc.robot.Lib.vision.GoalTracker;
import frc.robot.Lib.vision.TargetInfo;
import frc.robot.Utilities.RobotRelativeSpeed;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.Utilities.Geometry.Rotation2d;
import frc.robot.Utilities.Geometry.Translation2d;
import frc.robot.Utilities.RamseteTrajectory.DifferentialDriveKinematics;
import frc.robot.Utilities.RamseteTrajectory.DifferentialDriveWheelSpeeds;
import frc.robot.Utilities.RamseteTrajectory.Trajectory.State;
import frc.robot.subsystems.Vision;


public class RobotState {

    private static final RobotState instance = new RobotState();

    private DifferentialDriveKinematics kinematics;

    private GoalTracker vision_target = new GoalTracker();

    List<Translation2d> mCameraToVisionTargetPoses = new ArrayList<>();

    private Pose2d mCurrentPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private Pose2d initialPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

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


    public synchronized void resetVision() {
        vision_target.reset();
    }


    public synchronized void updateFieldToRobotPose( Pose2d pose ) {

     mCurrentPose = pose.relativeTo(initialPose);

    //  mCurrentPose = new Pose2d(pose.x() + initialPose.x(), 
    //                             pose.y() + initialPose.y(), 
    //                             Rotation2d.fromDegrees(pose.getRotation().getDegrees() + initialPose.getRotation().getDegrees()));

        //mCurrentPose = pose;
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

    public synchronized Pose2d getFieldToShooter(double timestamp) {
        return getFieldToVehicleInches().transformBy(Pose2d.fromRotation(Rotation2d.fromDegrees(0)));
    }

    private Translation2d getCameraToVisionTargetPose(TargetInfo target, boolean high, Vision source) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(source.getHorizontalPlaneToLens());
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = source.getLensHeight() - TechConstants.kHubTargetHeight;
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }

        return null;
    }


    private void updatePortGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPoses, GoalTracker tracker, Vision source) {
        if (cameraToVisionTargetPoses.size() != 2 ||
                cameraToVisionTargetPoses.get(0) == null ||
                cameraToVisionTargetPoses.get(1) == null) return;
        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetPoses.get(0).interpolate(
                cameraToVisionTargetPoses.get(1), 0.5));

        Pose2d fieldToVisionTarget = getFieldToShooter(timestamp).transformBy(source.getShooterToLens()).transformBy(cameraToVisionTarget);
        tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), Rotation2d.identity())));
    }

    /**
     * 
     * @param timestamp - timestamp
     * @param observations - list of observations
     * @param source - Vision object
     */
    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Vision source) {
        mCameraToVisionTargetPoses.clear();

        if (observations == null || observations.isEmpty()) {
            vision_target.update(timestamp, new ArrayList<>());
            return;
        }

        for (TargetInfo target : observations) {
            mCameraToVisionTargetPoses.add(getCameraToVisionTargetPose(target, false, source));
        }

        updatePortGoalTracker(timestamp, mCameraToVisionTargetPoses, vision_target, source);
    }
}
