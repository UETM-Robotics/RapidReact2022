package frc.robot.Utilities.Drivers;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriverConstants;

public class ControllerU extends XboxController {

    private boolean[] prevButtonVal;

    public ControllerU(int port) {
        super(port);
        
        prevButtonVal = new boolean[16];

        for(int i = 0; i < prevButtonVal.length; i++)
            prevButtonVal[i] = false;
    }

    public synchronized boolean getRisingEdgeButton(int button) {

        try {

            boolean currentButton= super.getRawButton(button);
            boolean retVal = (currentButton != prevButtonVal[button - 1] && currentButton);

            prevButtonVal[button - 1] = currentButton;
            return retVal;

        } catch(Exception e) {

            return false;

        }
    }

    
    public synchronized boolean getFallingEdgeButton(int button) {

        try {

            boolean currentButton= super.getRawButton(button);
            boolean retVal = (currentButton != prevButtonVal[button - 1] && !currentButton);

            prevButtonVal[button - 1] = currentButton;
            return retVal;

        } catch(Exception e) {

            return false;

        }
    }

    public double getNormalizedAxis(int axis, double deadband) {
		return normalizeJoystickWithDeadband(getRawAxis(axis), deadband);
	}

    private double normalizeJoystickWithDeadband(double val, double deadband) {
		val = (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;

		if (val != 0)
			val = Math.signum(val) * ((Math.abs(val) - deadband) / (1.0 - deadband));

		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

    public synchronized double getRawYAxis() {
        return super.getRawAxis(DriverConstants.DRIVE_Y_AXIS);
    }
    public synchronized double getTransformedYAxis() {
        return  Math.pow( Math.abs(super.getRawAxis(DriverConstants.DRIVE_Y_AXIS)), DriverConstants.joyYMod ) * (super.getRawAxis(DriverConstants.DRIVE_Y_AXIS) < 0 ? -1 : 1);
    }


    public synchronized double getRawXAxis() {
        return super.getRawAxis(DriverConstants.DRIVE_X_AXIS);
    }
    public synchronized double getTransformedXAxis() {
        return Math.pow( Math.abs(super.getRawAxis(DriverConstants.DRIVE_X_AXIS)), DriverConstants.joyXMod ) * (super.getRawAxis(DriverConstants.DRIVE_X_AXIS) < 0 ? -1 : 1);
    }
}
