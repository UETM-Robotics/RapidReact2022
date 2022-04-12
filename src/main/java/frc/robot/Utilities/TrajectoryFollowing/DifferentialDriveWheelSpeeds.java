package frc.robot.Utilities.TrajectoryFollowing;

/** Represents the wheel speeds for a differential drive drivetrain. */
@SuppressWarnings("MemberName")
public class DifferentialDriveWheelSpeeds {
    /** Speed of the left side of the robot. */
    public double leftMetersPerSecond;

    /** Speed of the right side of the robot. */
    public double rightMetersPerSecond;

    /** Constructs a DifferentialDriveWheelSpeeds with zeros for left and right speeds. */
    public DifferentialDriveWheelSpeeds() {}

    /**
     * Constructs a DifferentialDriveWheelSpeeds.
     *
     * @param leftMetersPerSecond The left speed.
     * @param rightMetersPerSecond The right speed.
     */
    public DifferentialDriveWheelSpeeds(double leftMetersPerSecond, double rightMetersPerSecond) {
        this.leftMetersPerSecond = leftMetersPerSecond;
        this.rightMetersPerSecond = rightMetersPerSecond;
    }

    /**
     * Renormalizes the wheel speeds if any either side is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more wheels may be
     * above the max attainable speed for the driving motor on that wheel. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between wheels.
     *
     * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a wheel can reach.
     */
    public void desaturate(double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = Math.max(Math.abs(leftMetersPerSecond), Math.abs(rightMetersPerSecond));

        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
        leftMetersPerSecond = leftMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
        rightMetersPerSecond =
            rightMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
        }
    }

    @Override
    public String toString() {
        return String.format(
            "DifferentialDriveWheelSpeeds(Left: %.2f m/s, Right: %.2f m/s)",
            leftMetersPerSecond, rightMetersPerSecond);
    }
}