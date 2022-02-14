package frc.robot.Actions.OperatedActions;

import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Intake;

public class SetIntakeAction implements Action {

    private static final Intake intake = Intake.getInstance();

    private final Supplier<Boolean> mButtonGetterMethod;
	private final Intake.ControlMode mControlMode;

    public SetIntakeAction(boolean reverse, Supplier<Boolean> buttonGetterMethod) {
		mButtonGetterMethod = buttonGetterMethod;
		mControlMode = reverse ? Intake.ControlMode.REVERSE : Intake.ControlMode.ENABLED;

        if(reverse) {
            intake.prepareToEject();
        } else {
            intake.revert();
        }
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
        intake.setControlMode(Intake.ControlMode.DISABLED);
    }

    @Override
    public void start() {
        intake.setControlMode(mControlMode);
    }


    
}
