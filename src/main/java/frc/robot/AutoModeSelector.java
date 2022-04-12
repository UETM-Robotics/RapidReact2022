package frc.robot;

import frc.robot.Autonomous.Framework.AutoModeBase;

import frc.robot.Autonomous.Modes.Test.Test;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        DEFAULT_START_POSITION;
    }

    enum DesiredMode {
        TEST;
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Wherever", StartingPosition.DEFAULT_START_POSITION);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);


        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Test", DesiredMode.TEST);

        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    //update choosers within loops until auto starts
    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;


    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {

        if( mode == DesiredMode.TEST) {
            return Optional.of(new Test());
        }

        switch(position) {

            case DEFAULT_START_POSITION:
                
                switch(mode) {
                    case TEST:
                    
                        return Optional.of(new Test());
                        
                    default:
                        return Optional.of(new Test());

                }
                
            default:
                break;

        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        return mAutoMode;
    }
}