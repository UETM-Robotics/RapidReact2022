package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class DriveTrainSim extends SubsystemBase {

    private final PWMSparkMax left;

    private final PWMSparkMax right;

    private final EncoderSim encLeft;

    private final EncoderSim encRight;


    private AnalogGyroSim gyro;


    private static DriveTrainSim instance;


    private final DifferentialDrivetrainSim driveSim;


    public static DriveTrainSim getInstance() {
        if(instance == null) {
            instance = new DriveTrainSim();
        }

        return instance;
    }

    private DriveTrainSim() {

        left = new PWMSparkMax(PortConstants.frontLeftPort);

        right = new PWMSparkMax(PortConstants.frontRightPort);

        encLeft = new EncoderSim( new Encoder(1, 2) );
        encRight = new EncoderSim( new Encoder(3, 4) );

        gyro = new AnalogGyroSim( new AnalogGyro(1) );

        driveSim = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDoubleNEOPerSide, 
            KitbotGearing.k10p71, 
            KitbotWheelSize.kSixInch, 
            null
        );

    }

    @Override
    public void simulationPeriodic() {

        driveSim.setInputs( left.get() * RobotController.getInputVoltage(), right.get() * RobotController.getInputVoltage() );

        driveSim.update(0.02);

        encLeft.setDistance( driveSim.getLeftPositionMeters() );
        encLeft.setRate(driveSim.getLeftVelocityMetersPerSecond());

        encRight.setDistance( driveSim.getRightPositionMeters() );
        encRight.setDistance(driveSim.getRightVelocityMetersPerSecond());
        gyro.setAngle(driveSim.getHeading().getDegrees());

    }



}
