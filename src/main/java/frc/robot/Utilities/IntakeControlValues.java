package frc.robot.Utilities;

public class IntakeControlValues {
    
    final double magnitude;

    public IntakeControlValues(double magnitude) {
        this.magnitude = magnitude;
    }

    public double getMagnitude() {
        return magnitude;
    }

    public static final IntakeControlValues NEUTRAL = new IntakeControlValues(0.0);

}
