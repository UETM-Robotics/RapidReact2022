package frc.robot.Utilities;


import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Utilities.Drivers.CANSpeedControllerBuilder;
import frc.robot.Utilities.Drivers.ControllerU;
import frc.robot.Utilities.Drivers.NavX;
import frc.robot.Utilities.Drivers.SparkMaxU;

public class Controllers {

    private static Controllers instance = null;
    public final boolean ATTEMPT_SIGMA_INIT = false;

    public static Controllers getInstance() {
        if(instance == null) {
            instance = new Controllers();
        }

        return instance;
    }

    private Controllers() {

        leftDriveFront = CANSpeedControllerBuilder.createFastMasterSparkMax(Constants.PortConstants.frontLeftPort, 0);
        leftDriveHind = CANSpeedControllerBuilder.createPermanentSlaveSparkMax(Constants.PortConstants.hindLeftPort, leftDriveFront);

        rightDriveFront = CANSpeedControllerBuilder.createFastMasterSparkMax(Constants.PortConstants.frontRightPort, 0);
        rightDriveHind = CANSpeedControllerBuilder.createPermanentSlaveSparkMax(Constants.PortConstants.hindRightPort, rightDriveFront);

        shooterMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(Constants.PortConstants.shooterPort, 0);
        beltTransportMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(Constants.PortConstants.beltTransportPort, 0);

        if(ATTEMPT_SIGMA_INIT) {
            hoodMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(Constants.PortConstants.hoodPort, 0);
        } else {
            hoodMotor = null;
        }

        throttleJoystick = new ControllerU(0);

        try {
            navX = new NavX(SPI.Port.kMXP);
        } catch (Exception ex) {
            
        }
    }

    //MOTORS
    //DRIVETRAIN MOTORS
    private SparkMaxU leftDriveFront;
    private SparkMaxU leftDriveHind;

    private SparkMaxU rightDriveFront;
    private SparkMaxU rightDriveHind;

    //SHOOTER MOTORS
    private SparkMaxU shooterMotor;
    private SparkMaxU beltTransportMotor;
    private SparkMaxU hoodMotor;


    //GYROSCOPE
    private NavX navX;


    //JOYSTICKS
    private ControllerU throttleJoystick;

    public SparkMaxU getLeftFrontDrive() {
        return leftDriveFront;
    }

    public SparkMaxU getRightFrontDrive() {
        return rightDriveFront;
    }

    public SparkMaxU getLeftHindDrive() {
        return leftDriveHind;
    }

    public SparkMaxU getRightHindDrive() {
        return rightDriveHind;
    }

    public SparkMaxU getShooterMotor() {
        return shooterMotor;
    }

    public SparkMaxU getBeltTransportMotor() {
        return beltTransportMotor;
    }

    public SparkMaxU getHoodMotor() {
        return hoodMotor;
    }

    public NavX getNavX() {
        return navX;
    }

    public ControllerU getThrottleJoystick() {
        return throttleJoystick;
    }
    
}
