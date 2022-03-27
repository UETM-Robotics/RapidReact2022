package frc.robot.Actions;

import frc.robot.Actions.Framework.Action;
import frc.robot.Autonomous.Paths.PathContainer;
import frc.robot.Utilities.TheoreticallyChadTrajectory.control.Path;
import frc.robot.subsystems.DriveTrain;

public class DrivePathAction implements Action {

    private PathContainer mPathContainer;
    private Path mPath;
    private DriveTrain dTrain = DriveTrain.getInstance();
    private boolean mStopWhenDone;

    public DrivePathAction(PathContainer p, boolean stopWhenDone) {
        mPathContainer = p;
        mPath = mPathContainer.buildPath();
        mStopWhenDone = stopWhenDone;
    }

    public DrivePathAction(PathContainer p) {
        this(p, false);
    }

    @Override
    public void start() {
        //dTrain.setWantDrivePath(mPath, mPathContainer.isReversed());
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        //return dTrain.isDoneWithPath();
        return true;
    }

    @Override
    public void done() {
        if (mStopWhenDone) {
            //dTrain.setVelocity(new DriveSignal(0, 0), new DriveSignal(0, 0));
        }
    }
}