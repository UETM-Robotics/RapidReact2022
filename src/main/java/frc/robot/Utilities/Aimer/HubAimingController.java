package frc.robot.Utilities.Aimer;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class HubAimingController {

    public static final HubAimingController instance = new HubAimingController();

    public static HubAimingController getInstance() {
        return instance;
    }

    private HubAimingController() {

    }
    

    private final DriveTrain dTrain = DriveTrain.getInstance();
    private final Vision vision = Vision.getInstance();

}
