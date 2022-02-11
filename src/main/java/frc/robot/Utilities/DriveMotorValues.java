package frc.robot.Utilities;

import frc.robot.Utilities.TrajectoryFollowingMotion.DriveSignal;

public class DriveMotorValues {
	public double leftDrive;
	public double rightDrive;
	
	public DriveMotorValues(double leftDrive, double rightDrive) {
		this.leftDrive = leftDrive;
		this.rightDrive = rightDrive;
	}

	public static DriveMotorValues fromDriveSignal(DriveSignal signal) {
		return new DriveMotorValues(signal.getLeft(), signal.getRight());
	}

	public static DriveMotorValues NEUTRAL = new DriveMotorValues(0, 0);
}
