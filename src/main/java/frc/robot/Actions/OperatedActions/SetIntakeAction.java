package frc.robot.Actions.OperatedActions;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeControlMode;

import java.util.function.Supplier;

public class SetIntakeAction implements Action {

    private final Supplier<Boolean> mButtonGetterMethod;
    private final Intake intake;

    private final IntakeControlMode mControlMode;

    public SetIntakeAction(boolean reverse, Supplier<Boolean> buttonGetterMethod) {

        intake = Intake.getInstance();

        mButtonGetterMethod = buttonGetterMethod;

        mControlMode = reverse ? IntakeControlMode.REVERSE : IntakeControlMode.ENABLED;

        if(reverse) 
            intake.prepareToEject();
        else
            intake.revert();
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
        
        intake.setControlMode(IntakeControlMode.DISABLED);
        
    }

    @Override
    public void start() {

        intake.setControlMode(mControlMode);

    }
    
}

