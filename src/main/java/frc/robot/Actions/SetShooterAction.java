package frc.robot.Actions;

import java.util.function.Supplier;

import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.BeltIndexerControlMode;
import frc.robot.subsystems.Shooter.HoodControlMode;
import frc.robot.subsystems.Shooter.ShooterControlMode;

public class SetShooterAction implements Action {

    private final boolean mIsAuto;
    private final int balls_to_shoot;
    private int ballsShot = 0;
    private final Supplier<Boolean> mButtonGetterMethod;

    public SetShooterAction(boolean isAuto, int ballsToShoot) {

        mIsAuto = isAuto;
        balls_to_shoot = ballsToShoot;
        mButtonGetterMethod = null;

    }

    public SetShooterAction(boolean isAuto, Supplier<Boolean> buttonGetterMethod) {
        mIsAuto = isAuto;
        balls_to_shoot = 0;
        mButtonGetterMethod = buttonGetterMethod;
    }


    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if(mIsAuto)
            return ballsShot == balls_to_shoot;
        else
            return (!mButtonGetterMethod.get());
    }

    @Override
    public void update() {
        
        if(Shooter.getInstance().getShooterVelocity() > Shooter.getInstance().getShooterVelocitySetpoint() * TechConstants.kShooterArmVelocityPercentage && mIsAuto) {
            Shooter.getInstance().setBeltIndexerControlMode(BeltIndexerControlMode.ENABLED);
        }
    }

    @Override
    public void done() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void start() {

        Shooter.getInstance().setShooterControlMode(ShooterControlMode.INTERPOLATING);
        Shooter.getInstance().setHoodControlMode(HoodControlMode.INTERPOLATING);

        Shooter.getInstance().setShooterControlMode(ShooterControlMode.VELOCITY);


        Shooter.getInstance().setShooterVelocitySetpoint(3500);
    }


    
}
