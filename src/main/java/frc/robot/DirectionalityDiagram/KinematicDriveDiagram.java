package frc.robot.DirectionalityDiagram;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class KinematicDriveDiagram extends CommandBase {

    private ArrayList<KinematicDriveSchematic> driveSchematics;

    public KinematicDriveDiagram(ArrayList<KinematicDriveSchematic> schematics) {
        
        driveSchematics = schematics;

    }
    

    @Override
    public void initialize() {

        for(KinematicDriveSchematic schematic : driveSchematics) {

            schematic.schedule();
            while(!schematic.isFinished()) {}

        }

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
