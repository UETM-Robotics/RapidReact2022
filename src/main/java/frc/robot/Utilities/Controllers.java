package frc.robot.Utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Utilities.Constants.PortConstants;
import frc.robot.Utilities.Drivers.CANSpeedControllerBuilder;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;

public class Controllers {

    private static final Controllers instance = new Controllers();

    public static Controllers getInstance() {
        return instance;
    }

    private Controllers() {
        
        leftFrontDriveMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.leftFrontDriveMotorPort, 0);
        leftHindDriveMotor = CANSpeedControllerBuilder.createPermanentSlaveSparkMax(PortConstants.leftHindDriveMotorPort, leftFrontDriveMotor);

        rightFrontDriveMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.rightFrontDriveMotorPort, 0);
        rightHindDriveMotor = CANSpeedControllerBuilder.createPermanentSlaveSparkMax(PortConstants.rightHindDriveMotorPort, rightFrontDriveMotor);

        gyro = new AHRS(SPI.Port.kMXP);


        driverController = new ControllerU(0);
    }

    private final SparkMaxU leftFrontDriveMotor;
    private final SparkMaxU leftHindDriveMotor;

    private final SparkMaxU rightFrontDriveMotor;
    private final SparkMaxU rightHindDriveMotor;

    private final AHRS gyro;
    

    private final ControllerU driverController;


    public SparkMaxU getLeftFrontDriveMotor() {
        return leftFrontDriveMotor;
    }

    public SparkMaxU getRightFrontDriveMotor() {
        return rightFrontDriveMotor;
    }

    public SparkMaxU getLeftHindDriveMotor() {
        return leftHindDriveMotor;
    }

    public SparkMaxU getRightHindDriveMotor() {
        return rightHindDriveMotor;
    }


    public AHRS getGyro() {
        return gyro;
    }

    public ControllerU getDriverController() {
        return driverController;
    }

}
