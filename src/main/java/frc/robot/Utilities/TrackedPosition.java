package frc.robot.Utilities;

public class TrackedPosition {

    double X;
    double Y;

    public TrackedPosition(double x, double y) {
        X = x;
        Y = y;
    }

    public void update(double x, double y) {
        X = x;
        Y = y;
    }

    public double getX() {
        return X;
    }

    public double getY() {
        return Y;
    }
    
}
