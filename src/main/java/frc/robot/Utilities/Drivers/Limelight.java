package frc.robot.Utilities.Drivers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = limelight.getEntry("tx");
    NetworkTableEntry ty = limelight.getEntry("ty");
    NetworkTableEntry tv = limelight.getEntry("tv");
    NetworkTableEntry ta = limelight.getEntry("ta");

    NetworkTableEntry pipelineNT = limelight.getEntry("pipeline");
    NetworkTableEntry objectDetected = limelight.getEntry("tv");
    NetworkTableEntry ledMode = limelight.getEntry("ledmode");
    NetworkTableEntry cameraMode = limelight.getEntry("camMode");

    private Pipeline currentPipeline;

    private LEDMode currentLEDMode;
    private CameraMode currentCamMode;


    public Limelight() {}


    public void setPipeline(Pipeline pipeline) {
        if(pipeline != currentPipeline) {

            currentPipeline = pipeline;
            
            switch(pipeline) {
                case TRACK_HUB:
                    pipelineNT.setNumber(0);
                    break;
                case TRACK_RED_CARGO:
                    pipelineNT.setNumber(1);
                    break;
                case TRACK_BLUE_CARGO:
                    pipelineNT.setNumber(2);
                    break;
                default:
                    pipelineNT.setNumber(0);
                    break;
                    
            }

        } else return;
    }

    public void setLED(LEDMode ledmode) {
        if(ledmode != currentLEDMode) {

            currentLEDMode = ledmode;

            switch(ledmode) {
                case FORCE_BLINK:
                    ledMode.setNumber(2);
                    break;
                case FORCE_OFF:
                    ledMode.setNumber(1);
                    break;
                case FORCE_ON:
                    ledMode.setNumber(3);
                    break;
                case PIPELINE_DEFAULT:
                    ledMode.setNumber(0);
                    break;
                default:
                    break;

            }

        } else return;
    }

    public void setDriverCam(CameraMode camMode) {

        if(currentCamMode != camMode) {

            currentCamMode = camMode;
            
            switch(camMode) {
                case DRIVER_CAMERA:
                    cameraMode.setNumber(1);
                    break;
                case VISION_PROCESSING:
                    cameraMode.setNumber(0);
                    break;
                default:
                    cameraMode.setNumber(0);
                    break;

            }

        } else return;

    }

    public Pipeline getCurrentPipeline() {
        return currentPipeline;
    }

    public LEDMode getCurrentLEDMode() {
        return currentLEDMode;
    }

    public CameraMode getCurrentCamMode() {
        return currentCamMode;
    }

    

    public double getTargetYOffset() {
        return ty.getDouble(-99);
    }

    public double getTargetXOffset() {
        return tx.getDouble(-99);
    }

    public double getTargetScreenArea() {
        return ta.getDouble(0);
    }

    public boolean hasTargets() {
        return tv.getNumber(0).intValue() == 1;
    }

    public enum Pipeline {
        TRACK_HUB,
        TRACK_RED_CARGO,
        TRACK_BLUE_CARGO   
    }
    
    public enum LEDMode {
        PIPELINE_DEFAULT,
        FORCE_OFF,
        FORCE_BLINK,
        FORCE_ON
    }

    public enum CameraMode {
        VISION_PROCESSING,
        DRIVER_CAMERA
    }
}
