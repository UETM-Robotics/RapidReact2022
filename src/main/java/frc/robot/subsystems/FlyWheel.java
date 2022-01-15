package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;

public class FlyWheel extends SubsystemBase
{
    private final CANSparkMax motorDom;
    private final CANSparkMax motorSub;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
    private final PIDController pidController = new PIDController(0.0, 0.0, 0.0);

    public FlyWheel()
    {
        motorDom = new CANSparkMax(PortConstants.mportDom, MotorType.kBrushless);
        motorSub = new CANSparkMax(PortConstants.mportSub, MotorType.kBrushless);

        motorDom.setInverted(true);
        motorSub.setInverted(true);

        motorDom.getEncoder().setVelocityConversionFactor(4 * Math.PI / 12 / 60);

        motorDom.setOpenLoopRampRate(1);
        motorSub.setOpenLoopRampRate(1);

        motorDom.setSmartCurrentLimit(45);
        motorSub.setSmartCurrentLimit(45);
    }

    public void Set(double speed)
    {
        motorDom.set(-speed);
        motorSub.set(speed);
    }

    public double get() {
        return motorDom.getEncoder().getVelocity();
    }

    public double getTemp() {
        return motorSub.getMotorTemperature();
    }
}
