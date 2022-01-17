package frc.robot.Utilities;


import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Utilities.Drivers.CANSpeedControllerBuilder;
import frc.robot.Utilities.Drivers.NavX;
import frc.robot.Utilities.Drivers.SparkMaxU;

public class Controllers {

    private static Controllers instance = null;

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

        try {
            navX = new NavX(SPI.Port.kMXP);
        } catch (Exception ex) {
            
        }
    }

    private SparkMaxU leftDriveFront;
    private SparkMaxU leftDriveHind;

    private SparkMaxU rightDriveFront;
    private SparkMaxU rightDriveHind;

    private NavX navX;

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

    public NavX getNavX() {
        return navX;
    }
    
}
