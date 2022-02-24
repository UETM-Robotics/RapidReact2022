package frc.robot.Actions.OperatedActions;

import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerControlMode;

public class SetIndexerAction implements Action {

    private final Supplier<Boolean> mButtonGetterMethod;
    private final Indexer indexer = Indexer.getInstance();
    private final IndexerControlMode mControlMode;

    public SetIndexerAction(boolean unjam, Supplier<Boolean> buttonGetterMethod) {

        mButtonGetterMethod = buttonGetterMethod;

        mControlMode = unjam ? IndexerControlMode.UNJAM : IndexerControlMode.ENABLED;

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
        indexer.setIndexerControlMode(IndexerControlMode.DISABLED);
    }

    @Override
    public void start() {
        indexer.setIndexerControlMode(mControlMode);
    }
    
}
