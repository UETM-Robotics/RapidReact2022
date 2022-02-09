package frc.robot.Utilities.Geometry;

import frc.robot.Utilities.CSVWritable;
import frc.robot.Utilities.TrajectoryFollowingMotion.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}