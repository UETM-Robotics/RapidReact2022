package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Utilities.Controllers;
import frc.robot.Utilities.CustomSubsystem;
import frc.robot.Utilities.ElapsedTimer;
import frc.robot.Utilities.Drivers.SparkHelper;
import frc.robot.Utilities.Drivers.SparkMaxU;
import frc.robot.Utilities.Loops.Looper;

import frc.robot.Utilities.Constants;

public class Shooter extends Subsystem implements CustomSubsystem {


    private static Shooter instance = new Shooter();

    private Controllers controllers = Controllers.getInstance();

    public static Shooter getInstance() {
        return instance;
    }

    private final SparkMaxU shooterMotor;
    private final SparkMaxU beltTransporterMotor;

    private ShooterControlMode mShooterControlMode = ShooterControlMode.SMART_VELOCITY;

    private PeriodicIO mPeriodicIO;

    private final ElapsedTimer loopTimer = new ElapsedTimer();
	private final ElapsedTimer shooter_dt = new ElapsedTimer();

    private Shooter() {

        shooterMotor = controllers.getShooterMotor();
        beltTransporterMotor = controllers.getBeltTransporterMotor();

    }


    @Override
    public void init() {
        
        // shooterMotor.setIdleMode(IdleMode.kCoast);
        // shooterMotor.setOpenLoopRampRate(0.2);
        // shooterMotor.setSmartCurrentLimit(60);

        // beltTransporterMotor.setIdleMode(IdleMode.kCoast);
        // beltTransporterMotor.setOpenLoopRampRate(0.5);
        // beltTransporterMotor.setSmartCurrentLimit(40);

        int retryCounter = 0;
        boolean setSucceeded;

        do {

            setSucceeded = true;

            setSucceeded &= SparkHelper.setPIDGains(shooterMotor, 0, Constants.kShooterVelocityKp, Constants.kShooterVelocityKi, Constants.kShooterVelocityKd, Constants.kShooterVelocityKf, Constants.kShooterVelocityClosedLoopRampRate, Constants.kShooterVelocityIZone);
            

        } while(retryCounter++ < 3 && !setSucceeded);

        if(retryCounter == 3) {
            DriverStation.reportError("Error Initializing Shooter", false);
        }
    }

    public void setChango() {
        //shooterMotor.set(0.3);
        beltTransporterMotor.set(1);
    }

    @Override
    public void subsystemHome() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void registerEnabledLoops(Looper in) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }


    @SuppressWarnings("WeakerAccess")
	public static class PeriodicIO {

		public double shooter_velocity_rpm;
		public double shooter_setpoint_rpm;

		// Outputs
		public double shooter_loop_time;
	}
    

    public enum ShooterControlMode {
		OPEN_LOOP,
		VELOCITY,
        SMART_VELOCITY,
		DISABLED;
	}
}
