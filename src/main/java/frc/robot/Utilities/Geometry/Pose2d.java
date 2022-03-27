package frc.robot.Utilities.Geometry;

import frc.robot.Utilities.Util;

public class Pose2d implements IPose2d<Pose2d>{

    protected static final Pose2d kIdentity = new Pose2d();

    public static final Pose2d identity() {
        return kIdentity;
    }

    private final static double kEps = 1E-9;

    protected final Translation2d translation_;
    protected final Rotation2d rotation_;

    public Pose2d() {
        translation_ = new Translation2d();
        rotation_ = new Rotation2d();
    }

    public Pose2d(double x, double y, final Rotation2d rotation) {
        translation_ = new Translation2d(x, y);
        rotation_ = rotation;
    }

    public Pose2d(final Translation2d translation, final Rotation2d rotation) {
        translation_ = translation;
        rotation_ = rotation;
    }

    public Pose2d(final Pose2d other) {
        translation_ = new Translation2d(other.translation_);
        rotation_ = new Rotation2d(other.rotation_);
    }

    public static Pose2d fromWpiLibPose2d( edu.wpi.first.math.geometry.Pose2d pose ) {
        return new Pose2d( pose.getX(), pose.getX(), Rotation2d.fromDegrees(pose.getRotation().getDegrees()) );
    }

    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, new Rotation2d());
    }

    public static Pose2d fromRotation(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    /**
     * Obtain a new Pose2d from a (constant curvature) velocity. See:
     * https://github.com/strasdat/Sophus/blob/master/sophus/se2.hpp
     */

    public Pose2d exp(Twist2d twist) {
        double dx = twist.dx;
        double dy = twist.dy;
        double dtheta = twist.dtheta;
    
        double sinTheta = Math.sin(dtheta);
        double cosTheta = Math.cos(dtheta);
    
        double s;
        double c;
        if (Math.abs(dtheta) < 1E-9) {
          s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
          c = 0.5 * dtheta;
        } else {
          s = sinTheta / dtheta;
          c = (1 - cosTheta) / dtheta;
        }
        var transform =
            new Transform2d(
                new Translation2d(dx * s - dy * c, dx * c + dy * s),
                new Rotation2d(cosTheta, sinTheta));
    
        return this.plus(transform);
      }

    /**
     * Logical inverse of the above.
     */
    public static Twist2d log(final Pose2d transform) {
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().cos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEps) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new Twist2d(translation_part.x(), translation_part.y(), dtheta);
    }

    public Transform2d minus(Pose2d other) {
        final var pose = this.relativeTo(other);
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public Pose2d plus(Transform2d other) {
        return transformBy(other);
      }

    @Override
    public Translation2d getTranslation() {
        return translation_;
    }

    @Override
    public Rotation2d getRotation() {
        return rotation_;
    }

    /**
     * Transforming this RigidTransform2d means first translating by other.translation and then rotating by
     * other.rotation
     *
     * @param other The other transform.
     * @return This transform * other
     */
    @Override
    public Pose2d transformBy(final Pose2d other) {
        return new Pose2d(translation_.translateBy(other.translation_.rotateBy(rotation_)),
                rotation_.rotateBy(other.rotation_));
    }

    /**
     * The inverse of this transform "undoes" the effect of translating by this transform.
     *
     * @return The opposite of this transform.
     */
    public Pose2d inverse() {
        Rotation2d rotation_inverted = rotation_.inverse();
        return new Pose2d(translation_.inverse().rotateBy(rotation_inverted), rotation_inverted);
    }

    public Pose2d normal() {
        return new Pose2d(translation_, rotation_.normal());
    }

    /**
     * Finds the point where the heading of this pose intersects the heading of another. Returns (+INF, +INF) if
     * parallel.
     */
    public Translation2d intersection(final Pose2d other) {
        final Rotation2d other_rotation = other.getRotation();
        if (rotation_.isParallel(other_rotation)) {
            // Lines are parallel.
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        if (Math.abs(rotation_.cos()) < Math.abs(other_rotation.cos())) {
            return intersectionInternal(this, other);
        } else {
            return intersectionInternal(other, this);
        }
    }

    /**
     * Return true if this pose is (nearly) colinear with the another.
     */
    public boolean isColinear(final Pose2d other) {
        if (!getRotation().isParallel(other.getRotation()))
            return false;
        final Twist2d twist = log(inverse().transformBy(other));
        return (Util.epsilonEquals(twist.dy, 0.0) && Util.epsilonEquals(twist.dtheta, 0.0));
    }

    public boolean epsilonEquals(final Pose2d other, double epsilon) {
        return getTranslation().epsilonEquals(other.getTranslation(), epsilon)
                && getRotation().isParallel(other.getRotation());
    }

    private static Translation2d intersectionInternal(final Pose2d a, final Pose2d b) {
        final Rotation2d a_r = a.getRotation();
        final Rotation2d b_r = b.getRotation();
        final Translation2d a_t = a.getTranslation();
        final Translation2d b_t = b.getTranslation();

        final double tan_b = b_r.tan();
        final double t = ((a_t.x() - b_t.x()) * tan_b + b_t.y() - a_t.y())
                / (a_r.sin() - a_r.cos() * tan_b);
        if (Double.isNaN(t)) {
            return new Translation2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }
        return a_t.translateBy(a_r.toTranslation().scale(t));
    }

    /**
     * Do twist interpolation of this pose assuming constant curvature.
     */
    @Override
    public Pose2d interpolate(final Pose2d other, double x) {
        if (x <= 0) {
            return new Pose2d(this);
        } else if (x >= 1) {
            return new Pose2d(other);
        }
        final Twist2d twist = Pose2d.log(inverse().transformBy(other));
        return transformBy(this.exp(twist.scaled(x)));
    }

    @Override
    public String toString() {
        return "T:" + translation_.toString() + ", R:" + rotation_.toString();
    }

    @Override
    public String toCSV() {
        return "";
    }

    @Override
    public double distance(final Pose2d other) {
        return Pose2d.log(inverse().transformBy(other)).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Pose2d)) return false;
        return epsilonEquals((Pose2d) other, Util.kEpsilon);
    }

    public Pose2d relativeTo(Pose2d other) {
        var transform = new Transform2d(other, this);
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    @Override
    public Pose2d getPose() {
        return this;
    }

    public Pose2d transformBy(Transform2d other) {
        return new Pose2d(
            translation_.plus(other.getTranslation().rotateBy(rotation_)),
            rotation_.plus(other.getRotation())
        );
    }


    public double x() {
        return translation_.x();
    }

    public double y() {
        return translation_.y();
    }


    @Override
    public Pose2d mirror() {
        return new Pose2d(new Translation2d(getTranslation().x(), -getTranslation().y()), getRotation().inverse());
    }
    
}
