package frc.robot.Utilities;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Utilities.Constants.PortConstants;
import frc.robot.Utilities.Drivers.CANSpeedControllerBuilder;
//import frc.robot.Utilities.Drivers.NavX;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Drivers.rcinput.ControllerU;

public class Controllers {

    private static Controllers instance = new Controllers();

    public static Controllers getInstance() {
        return instance;
    }

    private Controllers() {

        //Instantiate Drive Motors
        leftFront = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.leftFrontMotorPort, 0);
        leftHind = CANSpeedControllerBuilder.createPermanentSlaveSparkMax(PortConstants.leftHindMotorPort, leftFront);

        rightFront = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.rightFrontMotorPort, 0);
        rightHind = CANSpeedControllerBuilder.createPermanentSlaveSparkMax(PortConstants.rightHindMotorPort, rightFront);

        intakeMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.intakeMotorPort, 0);

        shooterMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.shooterMotorPort, 0);
        hoodMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.hoodMotorPort, 0);
        beltIndexerMotor = CANSpeedControllerBuilder.createFastMasterSparkMax(PortConstants.beltIndexerMotorPort, 0);


        driverController = new ControllerU(PortConstants.driverControllerPort);
        operatorController = new ControllerU(PortConstants.operatorControllerPort);

        //Instantiate Gyro
        //navX = new NavX(SPI.Port.kMXP);
        //TODO: Just in case of more gyro funny business
        navX = new AHRS(SPI.Port.kMXP);
    }

    //Drive Motors
    private final SparkMaxU leftFront, rightFront;
    private final SparkMaxU leftHind, rightHind;

    //Intake Motor
    private final SparkMaxU intakeMotor;

    //Shooter Motors
    private final SparkMaxU shooterMotor;
    private final SparkMaxU hoodMotor;
    private final SparkMaxU beltIndexerMotor;


    //Gyro
    //private final NavX navX;
    //TODO: Just in case of more gyro funny business
    private final AHRS navX;


    //Controllers
    private final ControllerU driverController;
    private final ControllerU operatorController;

    
    public SparkMaxU getLeftFrontDriveMotor() {
        return leftFront;
    }

    public SparkMaxU getRightFrontDriveMotor() {
        return rightFront;
    }

    public SparkMaxU getLeftHindDriveMotor() {
        return leftHind;
    }

    public SparkMaxU getRightHindDriveMotor() {
        return rightHind;
    }


    public SparkMaxU getIntakeMotor() {
        return intakeMotor;
    }


    public SparkMaxU getShooterMotor() {
        return shooterMotor;
    }

    public SparkMaxU getHoodMotor() {
        return hoodMotor;
    }

    public SparkMaxU getBeltIndexerMotor() {
        return beltIndexerMotor;
    }


    // public NavX getNavX() {
    //     return navX;
    // }

    public AHRS getNavX() {
        return navX;
    }


    public ControllerU getDriverController() {
        return driverController;
    }

    public ControllerU getOperatorController() {
        return operatorController;
    }
    
}
