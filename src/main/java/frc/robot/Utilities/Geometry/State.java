package frc.robot.Utilities.Geometry;

import frc.robot.Utilities.Interpolable;

public interface State<S> extends Interpolable<S> {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}