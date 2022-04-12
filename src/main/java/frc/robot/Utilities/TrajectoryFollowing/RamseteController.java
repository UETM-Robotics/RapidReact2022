package frc.robot.Utilities.TrajectoryFollowing;

import frc.robot.Utilities.RobotRelativeSpeed;
import frc.robot.Utilities.Geometry.Pose2d;

public class RamseteController {

  private static final RamseteController instance = new RamseteController();

  public static RamseteController getInstance() {
    return instance;
  }
    
    @SuppressWarnings("MemberName")
    private final double m_b;

    @SuppressWarnings("MemberName")
    private final double m_zeta;

    private Pose2d m_poseError = new Pose2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private boolean m_enabled = true;

    /**
    * Construct a Ramsete unicycle controller.
    *
    * @param b Tuning parameter (b &gt; 0 rad²/m²) for which larger values make convergence more
    *     aggressive like a proportional term.
    * @param zeta Tuning parameter (0 rad⁻¹ &lt; zeta &lt; 1 rad⁻¹) for which larger values provide
    *     more damping in response.
    */
    @SuppressWarnings("ParameterName")
    private RamseteController(double b, double zeta) {
        m_b = b;
        m_zeta = zeta;
    }

    private RamseteController() {
        this(2.0, 0.7);
    }

      /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        final var eTranslate = m_poseError.getTranslation();
        final var eRotate = m_poseError.getRotation();
        final var tolTranslate = m_poseTolerance.getTranslation();
        final var tolRotate = m_poseTolerance.getRotation();
        return Math.abs(eTranslate.x()) < tolTranslate.x()
            && Math.abs(eTranslate.y()) < tolTranslate.y()
            && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

      /**
     * Sets the pose error which is considered tolerable for use with atReference().
     *
     * @param poseTolerance Pose error which is tolerable.
     */
    public void setTolerance(Pose2d poseTolerance) {
        m_poseTolerance = poseTolerance;
    }

    /**
   * Returns the next output of the Ramsete controller.
   *
   * <p>The reference pose, linear velocity, and angular velocity should come from a drivetrain
   * trajectory.
   *
   * @param currentPose The current pose.
   * @param poseRef The desired pose.
   * @param linearVelocityRefMeters The desired linear velocity in meters per second.
   * @param angularVelocityRefRadiansPerSecond The desired angular velocity in radians per second.
   * @return The next controller output.
   */
    @SuppressWarnings("LocalVariableName")
    public RobotRelativeSpeed calculate(
        Pose2d currentPose,
        Pose2d poseRef,
        double linearVelocityRefMeters,
        double angularVelocityRefRadiansPerSecond) {
        if (!m_enabled) {
            return new RobotRelativeSpeed(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
        }

        m_poseError = poseRef.relativeTo(currentPose);

        // Aliases for equation readability
        final double eX = m_poseError.x();
        final double eY = m_poseError.y();
        final double eTheta = m_poseError.getRotation().getRadians();
        final double vRef = linearVelocityRefMeters;
        final double omegaRef = angularVelocityRefRadiansPerSecond;

        double k = 2.0 * m_zeta * Math.sqrt(Math.pow(omegaRef, 2) + m_b * Math.pow(vRef, 2));

        return new RobotRelativeSpeed(
            vRef * m_poseError.getRotation().cos() + k * eX,
            0.0,
            omegaRef + k * eTheta + m_b * vRef * sinc(eTheta) * eY);
    }


    /**
     * Returns the next output of the Ramsete controller.
     *
     * <p>The reference pose, linear velocity, and angular velocity should come from a drivetrain
     * trajectory.
     *
     * @param currentPose The current pose.
     * @param desiredState The desired pose, linear velocity, and angular velocity from a trajectory.
     * @return The next controller output.
     */
    @SuppressWarnings("LocalVariableName")
    public RobotRelativeSpeed calculate(Pose2d currentPose, Trajectory.State desiredState) {
        return calculate(
            currentPose,
            desiredState.poseMeters,
            desiredState.velocityMetersPerSecond,
            desiredState.velocityMetersPerSecond * desiredState.curvatureRadPerMeter);
    }

  /**
   * Enables and disables the controller for troubleshooting purposes.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }

  /**
   * Returns sin(x) / x.
   *
   * @param x Value of which to take sinc(x).
   */
  @SuppressWarnings("ParameterName")
  private static double sinc(double x) {
    if (Math.abs(x) < 1e-9) {
      return 1.0 - 1.0 / 6.0 * x * x;
    } else {
      return Math.sin(x) / x;
    }
  }
}