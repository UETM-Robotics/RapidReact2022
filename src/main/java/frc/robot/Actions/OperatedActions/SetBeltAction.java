package frc.robot.Actions.OperatedActions;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeControlMode;
import frc.robot.subsystems.Shooter.BeltIndexerControlMode;
import frc.robot.subsystems.Shooter.ShooterControlMode;

import java.util.function.Supplier;

public class SetBeltAction implements Action {

    private final Supplier<Boolean> mButtonGetterMethod;
    private final Shooter shooter;

    private final BeltIndexerControlMode mControlMode;

    public SetBeltAction(boolean reverse, Supplier<Boolean> buttonGetterMethod) {

        shooter = Shooter.getInstance();

        mButtonGetterMethod = buttonGetterMethod;

        mControlMode = reverse ? BeltIndexerControlMode.REVERSED : BeltIndexerControlMode.ENABLED;

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (!mButtonGetterMethod.get());
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void done() {
        
        shooter.setBeltIndexerControlMode(BeltIndexerControlMode.DISABLED);
        
    }

    @Override
    public void start() {

        shooter.setBeltIndexerControlMode(mControlMode);

    }
    
}

