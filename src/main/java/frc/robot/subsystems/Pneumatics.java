package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Loops.Looper;
import frc.robot.Utilities.CustomSubsystem;

public class Pneumatics extends Subsystem implements CustomSubsystem {
    
    private static Pneumatics instance = new Pneumatics();

    public static Pneumatics getInstance() {
        return instance;
    }

    private final Compressor compressor;


    private Pneumatics() {

        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

    }

    public void setCompressor(boolean on) {
        if(on)
            compressor.enableDigital();
        else
            compressor.disable();
    }

    public boolean getCompressor() {
        return compressor.enabled();
    }


    @Override
    public void init() {
        // TODO Auto-generated method stub
        
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
    
}
