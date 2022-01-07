package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    private final CANSparkMax fl = new CANSparkMax(Constants.PortConstants.flPort, MotorType.kBrushless);
    private final CANSparkMax hl = new CANSparkMax(Constants.PortConstants.hlPort, MotorType.kBrushless);

    private final CANSparkMax fr = new CANSparkMax(Constants.PortConstants.frPort, MotorType.kBrushless);
    private final CANSparkMax hr = new CANSparkMax(Constants.PortConstants.hrPort, MotorType.kBrushless);

    private final SpeedControllerGroup lDriveSide = new SpeedControllerGroup(fl, hl);
    private final SpeedControllerGroup rDriveSide = new SpeedControllerGroup(fr, hr);

    private final DifferentialDrive drive = new DifferentialDrive(lDriveSide, rDriveSide);

    public DriveTrain() {

    }
    
}
