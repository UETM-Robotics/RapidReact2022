package frc.robot.Actions;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Actions.Framework.Action;
import frc.robot.Utilities.Constants.TechConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.BeltIndexerControlMode;
import frc.robot.subsystems.Shooter.HoodControlMode;
import frc.robot.subsystems.Shooter.ShooterControlMode;

public class SetShooterAction implements Action {

    private final boolean mIsAuto;
    private final int balls_to_shoot;

    private static final Shooter shooter = Shooter.getInstance();
    
    private final Supplier<Boolean> mButtonGetterMethod;

    private int ballsShot = 0;
    private boolean triggerArmed = false;
    private boolean triggerFired = false;

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
        
        if(mIsAuto) {

            triggerArmed = shooter.getShooterVelocity() > shooter.getShooterVelocitySetpoint() * TechConstants.kShooterArmVelocityPercentage;
            

            if(triggerArmed) {

                shooter.setBeltIndexerControlMode(BeltIndexerControlMode.ENABLED);
                triggerFired = shooter.getShooterVelocity() < shooter.getShooterVelocitySetpoint() * TechConstants.kShooterTriggerVelocityPercentage;
                
                if(triggerFired) {
                    SmartDashboard.putString("Trigger Status", "FIRED");
                    shooter.setBeltIndexerControlMode(BeltIndexerControlMode.DISABLED);
                    ballsShot++;
                } else {
                    SmartDashboard.putString("Trigger Status", "ARMED");
                }

            } else {
                SmartDashboard.putString("Trigger Status", "DISARMED");
            }
        }
        
    }

    @Override
    public void done() {
        shooter.setShooterControlMode(ShooterControlMode.DISABLED);
        shooter.setHoodControlMode(HoodControlMode.DISABLED);
        shooter.setBeltIndexerControlMode(BeltIndexerControlMode.DISABLED);
    }

    @Override
    public void start() {

        //TODO: commented out for debugging
        // shooter.setShooterControlMode(ShooterControlMode.INTERPOLATING);
        // shooter.setHoodControlMode(HoodControlMode.INTERPOLATING);

        shooter.setShooterControlMode(ShooterControlMode.VELOCITY);


        //TODO: USE THESE VALUES TO GENERATE POLYNOMIAL REGRESSION
        shooter.setShooterVelocitySetpoint(3500);
        shooter.setHoodPositionSetpoint(0);
    }


    
}
