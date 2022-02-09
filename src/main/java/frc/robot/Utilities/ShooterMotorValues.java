package frc.robot.Utilities;

public class ShooterMotorValues {

    public double magnitude;

    public ShooterMotorValues(double magnitude) {
        this.magnitude = magnitude;
    }
    
    public static ShooterMotorValues NEUTRAL = new ShooterMotorValues(0.0);
}
