package frc.robot.Autonomous.Paths;

import frc.robot.Utilities.TheoreticallyChadTrajectory.control.Path;

public interface PathContainer {
    Path buildPath();

    boolean isReversed();
}