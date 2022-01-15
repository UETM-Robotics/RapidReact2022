package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.PortConstants;
import frc.robot.Constants.TrajectoryConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends SubsystemBase {

    private final CANSparkMax fl;
    private final CANSparkMax hl;
    
    private final CANSparkMax fr;
    private final CANSparkMax hr;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final AHRS nav;

    private static DriveTrain instance;

    public Pose2d pose;
    public DifferentialDriveKinematics kinematics;
    public DifferentialDriveOdometry odometry;

    public SimpleMotorFeedforward feedforward;
    public PIDController leftController;
    public PIDController rightController;
    // public SparkMaxPIDController rightController;
    // public SparkMaxPIDController leftController;

    public NetworkTableEntry xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    public NetworkTableEntry yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
    public NetworkTableEntry theta = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Theta");


    private final AnalogGyroSim gyro;

    private final PWMSparkMax leftMotorSim;
    private final PWMSparkMax rightMotorSim;

    private final EncoderSim leftEncoderSim;
    private final EncoderSim rightEncoderSim;

    private final DifferentialDrivetrainSim drivetrainSim;

    private final DifferentialDriveOdometry odometrySim;



    public static DriveTrain getInstance() {
        if(instance == null) {
            instance = new DriveTrain();
        }

        return instance;
    }

    private DriveTrain() {

        fl = new CANSparkMax(PortConstants.frontLeftPort, MotorType.kBrushless);
        hl = new CANSparkMax(PortConstants.hindLeftPort, MotorType.kBrushless);

        fr = new CANSparkMax(PortConstants.frontRightPort, MotorType.kBrushless);
        hr = new CANSparkMax(PortConstants.hindRightPort, MotorType.kBrushless);

        fr.setInverted(true);
        hr.setInverted(true);

        fl.setInverted(false);
        hl.setInverted(false);

        setCoast();

        hl.follow(fl);
        hr.follow(fr);

        leftEncoder = fl.getEncoder();
        rightEncoder = fr.getEncoder();

        nav = new AHRS(Port.kMXP);

        leftEncoder.setVelocityConversionFactor( TrajectoryConstants.velocityConversionFactorMeters );
        rightEncoder.setVelocityConversionFactor( TrajectoryConstants.velocityConversionFactorMeters );

        leftEncoder.setPositionConversionFactor( TrajectoryConstants.positionConversionFactorMeters );
        rightEncoder.setPositionConversionFactor( TrajectoryConstants.positionConversionFactorMeters );

        kinematics = new DifferentialDriveKinematics( Units.inchesToMeters( TrajectoryConstants.robotTrackWidthInches ) );
        odometry = new DifferentialDriveOdometry(getHeading(), new Pose2d(0, 0, getHeading()) );

        feedforward = new SimpleMotorFeedforward(0.1476, 2.8719, 0.26283);
        // leftController = fl.getPIDController();
        // rightController = fr.getPIDController();
        leftController = new PIDController(3.3559, 0.0, 0.0);
        rightController = new PIDController(3.3559, 0.0, 0.0);

        // leftController.setP(3.3559);
        // rightController.setP(3.3559);
        
        

        pose = new Pose2d();

        if(!RobotBase.isReal()) {

            gyro = new AnalogGyroSim(0);

            leftMotorSim = new PWMSparkMax(0);
            rightMotorSim = new PWMSparkMax(1);

            leftEncoderSim = new EncoderSim( new Encoder(0, 1) );
            rightEncoderSim = new EncoderSim( new Encoder( 2, 3 ) );

            drivetrainSim = DifferentialDrivetrainSim.createKitbotSim(
                KitbotMotor.kDoubleNEOPerSide, 
                KitbotGearing.k10p71, 
                KitbotWheelSize.kSixInch, 
                null
            );

            odometrySim = new DifferentialDriveOdometry( new Rotation2d(gyro.getAngle()) );

        } else {

            gyro = null;

            leftMotorSim = null;
            rightMotorSim = null;

            leftEncoderSim = null;
            rightEncoderSim = null;

            drivetrainSim = null;

            odometrySim = null;

        }

    }

    public void invertDriveTrain(boolean inverted) {
        fl.setInverted(inverted);
        hl.setInverted(inverted);

        fr.setInverted(!inverted);
        hr.setInverted(!inverted);
    }

    public void setCoast() {
        fl.setIdleMode(IdleMode.kCoast);
        hl.setIdleMode(IdleMode.kCoast);

        fr.setIdleMode(IdleMode.kCoast);
        hr.setIdleMode(IdleMode.kCoast);
    }

    public void setBrake() {
        fl.setIdleMode(IdleMode.kBrake);
        hl.setIdleMode(IdleMode.kBrake);

        fr.setIdleMode(IdleMode.kBrake);
        hr.setIdleMode(IdleMode.kBrake);
    }

    public void driveTank(double l, double r) {

        l = Math.abs(l) < 0.08 ? 0 : l;
        r = Math.abs(r) < 0.08 ? 0 : r;

        fl.set(l);
        fr.set(r);
    }

    public void resetEncoder() {
        fl.getEncoder().setPosition(0);
        fr.getEncoder().setPosition(0);

        pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }

    public void resetPose() {
        pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }

    public void resetOdometry() {
        odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
    }

    public void resetGyro() {
        nav.reset();
    }
    

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(),
            rightEncoder.getVelocity()
        );
    }

    public void outputMPS(double l, double r) {
        SmartDashboard.putNumber("left speedMPS", l);
        SmartDashboard.putNumber("right speedMPS", r);

        SmartDashboard.putNumber("left speed", l / TrajectoryConstants.robotMaxVelocityMPS);
        SmartDashboard.putNumber("right speed", r / TrajectoryConstants.robotMaxVelocityMPS);

        fl.set(l / TrajectoryConstants.robotMaxVelocityMPS);
        fr.set(r / TrajectoryConstants.robotMaxVelocityMPS);
    }
    public Rotation2d getHeading() {
        return nav.getRotation2d();
    }
    
    public Pose2d getPose() {
        return pose;
    }

    public void setOutput(double leftVolts, double rightVolts) {
        // fl.set(-leftVolts);
        // fr.set(-rightVolts);

        fl.setVoltage(leftVolts);
        hl.setVoltage(leftVolts);
        fr.setVoltage(rightVolts);
        hr.setVoltage(rightVolts);
    }

    
    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    @Override
    public void periodic() {
        pose = odometry.update( getHeading() , leftEncoder.getPosition(), rightEncoder.getPosition());

        SmartDashboard.putNumber("left speed", fl.get());
        SmartDashboard.putNumber("right speed", fr.get());

        var translation = odometry.getPoseMeters();
        xEntry.setNumber(translation.getX());
        yEntry.setNumber(translation.getY());
        theta.setNumber( translation.getRotation().getDegrees() );
    }
    
    @Override
    public void simulationPeriodic() {

        drivetrainSim.setInputs( 
            leftMotorSim.get() * 12, 
            rightMotorSim.get() * 12 
        );

        drivetrainSim.update(0.02);

        leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());

        rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
        rightEncoderSim.setDistance(drivetrainSim.getRightVelocityMetersPerSecond());
        gyro.setAngle(-drivetrainSim.getHeading().getDegrees());

    }
}
