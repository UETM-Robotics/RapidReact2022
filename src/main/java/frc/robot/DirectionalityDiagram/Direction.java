package frc.robot.DirectionalityDiagram;

public class Direction {

    public double leftMetersTraveled;
    public double rightMetersTraveled;

    public Direction() {
        leftMetersTraveled = 0;
        rightMetersTraveled = 0;
    }

    public void update(double l, double r) {
        leftMetersTraveled = l;
        rightMetersTraveled = r;
    }
    
}
