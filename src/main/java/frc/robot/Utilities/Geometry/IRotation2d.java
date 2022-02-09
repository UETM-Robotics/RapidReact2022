package frc.robot.Utilities.Geometry;

import frc.robot.Utilities.TrajectoryFollowingMotion.Rotation2d;

public interface IRotation2d<S> extends State<S> {
    public Rotation2d getRotation();
}