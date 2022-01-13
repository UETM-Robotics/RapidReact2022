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
    }

    public void Set(double speed)
    {
        motorDom.set(speed);
        motorSub.set(speed);
    }
}
