package frc.robot.Actions.OperatedActions;

import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer.BeltControlMode;

public class SetBeltAction implements Action {

    private static final Indexer indexer = Indexer.getInstance();

    private final Supplier<Boolean> mButtonGetterMethod;
	private final BeltControlMode mControlMode;

    public SetBeltAction(boolean reverse, Supplier<Boolean> buttonGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mControlMode = reverse ? BeltControlMode.REVERSE : BeltControlMode.ENABLED;
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
        indexer.setBeltControlMode(BeltControlMode.DISABLED);
    }

    @Override
    public void start() {
        indexer.setBeltControlMode(mControlMode);
    }


    
}
