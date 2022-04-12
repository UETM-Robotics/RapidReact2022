package frc.robot.Utilities;

import frc.robot.Utilities.Geometry.Rotation2d;

public class RobotRelativeSpeed {

    /** Represents forward velocity w.r.t the robot frame of reference. (Fwd is +) */
    public double vxMetersPerSecond;

    /** Represents sideways velocity w.r.t the robot frame of reference. (Left is +) */
    public double vyMetersPerSecond;

    /** Represents the angular velocity of the robot frame. (CCW is +) */
    public double omegaRadiansPerSecond;

    /** Constructs a ChassisSpeeds with zeros for dx, dy, and theta. */
    public RobotRelativeSpeed() {}

    /**
     * Constructs a ChassisSpeeds object.
     *
     * @param vxMetersPerSecond Forward velocity.
     * @param vyMetersPerSecond Sideways velocity.
     * @param omegaRadiansPerSecond Angular velocity.
     */
    public RobotRelativeSpeed(
        double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this.vxMetersPerSecond = vxMetersPerSecond;
        this.vyMetersPerSecond = vyMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }

    /**
    * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
    * object.
    *
    * @param vxMetersPerSecond The component of speed in the x direction relative to the field.
    *     Positive x is away from your alliance wall.
    * @param vyMetersPerSecond The component of speed in the y direction relative to the field.
    *     Positive y is to your left when standing behind the alliance wall.
    * @param omegaRadiansPerSecond The angular rate of the robot.
    * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
    *     considered to be zero when it is facing directly away from your alliance station wall.
    *     Remember that this should be CCW positive.
    * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
    */
    public static RobotRelativeSpeed fromFieldRelativeSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, Rotation2d robotAngle) {
        return new RobotRelativeSpeed(
            vxMetersPerSecond * robotAngle.cos() + vyMetersPerSecond * robotAngle.sin(),
            -vxMetersPerSecond * robotAngle.sin() + vyMetersPerSecond * robotAngle.cos(),
            omegaRadiansPerSecond);
    }

    @Override
    public String toString() {
        return String.format(
            "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)",
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
}