package frc.robot.DirectionalityDiagram;

import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class RotationalDriveShematic extends KinematicDriveSchematic {

    private final double targetAngle;
    private final DriveTrain dTrain;

    private final PIDController pidTheta = new PIDController(0.0, 0.0, 0.0);

    private final PIDController pidVel = new PIDController(3.1998, 0.0, 0.0);
    private final SimpleMotorFeedforward feedforward= new SimpleMotorFeedforward(0.15275, 2.8485, 0.23021);

    private double setPointMeters;
    

    private double leftSetpoint = 0.0;
    private double rightSetpoint = 0.0;

    private double leftOutput = 0.0;
    private double rightOutput = 0.0;

    private Pose2d pose = new Pose2d();
    

    private final boolean useRotationalPID;
    private final boolean useVelocityPID;


    public RotationalDriveShematic(double targetAngle, DriveTrain dTrain) {

        super();

        this.targetAngle = targetAngle;
        this.dTrain = dTrain;

        dTrain = DriveTrain.getInstance();

        useRotationalPID = TrajectoryConstants.useRotationalPID;
        useVelocityPID = TrajectoryConstants.useVelocityPID;

    }

    @Override
    public void initialize() {

        dTrain.setBrake();

        setPointMeters = ( targetAngle * (Units.inchesToMeters(TrajectoryConstants.robotTrackWidthInches) * Math.PI) ) / 360.0;

    }

    @Override
    public void execute() {

        if(useRotationalPID) {

            //leftSetpoint = pidTheta.calculate(dTrain.get, setpoint)

        } else {

        }

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return dTrain.getHeading().getDegrees() == targetAngle;
    }
    
}
