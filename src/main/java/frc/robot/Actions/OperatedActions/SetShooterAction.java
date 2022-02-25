package frc.robot.Actions.OperatedActions;

import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Shooter;

public class SetShooterAction implements Action {

    private static final Shooter shooter = Shooter.getInstance();

    private final Supplier<Boolean> mButtonGetterMethod;

    public SetShooterAction(Supplier<Boolean> buttonGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
	}

    @Override
	public boolean isFinished() {
		return (!mButtonGetterMethod.get());
	}

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void done() {
        shooter.setShooterControlMode(Shooter.ShooterControlMode.DISABLED);
    }

    @Override
    public void start() {
        shooter.setShooterControlMode(Shooter.ShooterControlMode.SMART_VELOCITY);
        shooter.setShooterVelocity(3500);
    }


    
}
