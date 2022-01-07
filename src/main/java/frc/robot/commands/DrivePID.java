package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivePID extends CommandBase {

    private final double kP = 0.5;
    private final double kI = 0;
    private final double kD = 0;

    private boolean finished = false;

    private final PIDController controller = new PIDController(kP, kI, kD);

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
    }
    
}
