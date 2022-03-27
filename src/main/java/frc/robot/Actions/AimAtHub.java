package frc.robot.Actions;

import java.util.function.Supplier;

import frc.robot.RobotState;
import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.Utilities.Geometry.Pose2d;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveControlState;

public class AimAtHub implements Action {

    private final boolean mIsAuto;
    private final Supplier<Boolean> mButtonGetterMethod;

    
    /**
     * 
     * @param isAuto True when this command is run in autonomous mode
     */
    public AimAtHub(boolean isAuto) {
        mIsAuto = isAuto;
        mButtonGetterMethod = null;
    }


    public AimAtHub(boolean isAuto, Supplier<Boolean> buttonGetterMethod) {
        mIsAuto = isAuto;
        mButtonGetterMethod = buttonGetterMethod;
    }


    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub

        // if(mIsAuto)
        //     return DriveTrain.getInstance().isAimedAtHub();
        // else
        //     return (!mButtonGetterMethod.get());

        return true;
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void done() {
        
        if(mIsAuto)
            DriveTrain.getInstance().setControlMode(DriveControlState.PATH_FOLLOWING);
        else
            DriveTrain.getInstance().setControlMode(DriveControlState.DRIVER_CONTROL);

    }

    @Override
    public void start() {

        DriveTrain.getInstance().setControlMode(DriveControlState.AUTO_AIMING);
        
        Pose2d currPose = RobotState.getInstance().getFieldToVehicleMeters();

        DriveTrain.getInstance().setHubAimingGuess( 
            Math.atan2(TechConstants.kHubPositionYMeters - currPose.y(),
                        TechConstants.kHubPositionXMeters - currPose.x()) - currPose.getRotation().getDegrees());

    }


    
}
