package frc.robot;

import frc.robot.Autonomous.Framework.AutoModeBase;

import frc.robot.Autonomous.Modes.*;
import frc.robot.Autonomous.Modes.StartFromLeftTarmac.LeftLeftTwoBall;
import frc.robot.Autonomous.Modes.StartFromLeftTarmac.LeftRightTwoBall;
import frc.robot.Autonomous.Modes.StartFromRightTarmac.RightLeftThreeBall;
import frc.robot.Autonomous.Modes.StartFromRightTarmac.RightLeftTwoBall;
import frc.robot.Autonomous.Modes.StartFromRightTarmac.RightRightThreeBall;
import frc.robot.Autonomous.Modes.StartFromRightTarmac.RightRightTwoBall;
import frc.robot.Autonomous.Modes.Test.Test;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        LEFT_TARMAC_LEFT, LEFT_TARMAC_RIGHT, RIGHT_TARMAC_LEFT, RIGHT_TARMAC_RIGHT;
    }

    enum DesiredMode {
        TEST,
        ONE_BALL,
        TWO_BALL,
        THREE_BALL;
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Left Tarmac Left", StartingPosition.LEFT_TARMAC_LEFT);
        mStartPositionChooser.addOption("Left Tarmac Right", StartingPosition.LEFT_TARMAC_RIGHT);
        mStartPositionChooser.setDefaultOption("Right Tarmac Left", StartingPosition.RIGHT_TARMAC_LEFT);
        mStartPositionChooser.addOption("Right Tarmac Right", StartingPosition.RIGHT_TARMAC_RIGHT);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);


        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("One Ball", DesiredMode.ONE_BALL);
        mModeChooser.addOption("Two Ball", DesiredMode.TWO_BALL);
        mModeChooser.addOption("Three Ball", DesiredMode.THREE_BALL);
        mModeChooser.addOption("Test", DesiredMode.TEST);

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

            case LEFT_TARMAC_LEFT:
            
                switch(mode) {

                    case ONE_BALL:
                        return Optional.of(new OneBall());
                    case TWO_BALL:
                        return Optional.of(new LeftLeftTwoBall());
                    default:
                        return Optional.of(new OneBall());

                }
                
            case LEFT_TARMAC_RIGHT:

                switch(mode) {

                    case ONE_BALL:
                        return Optional.of(new OneBall());
                    case TWO_BALL:
                        return Optional.of(new LeftRightTwoBall());
                    default:
                        return Optional.of(new OneBall());

                }

            case RIGHT_TARMAC_LEFT:

                switch(mode) {

                    case ONE_BALL:
                        return Optional.of(new OneBall());
                    case TWO_BALL:
                        return Optional.of(new RightLeftTwoBall());
                    case THREE_BALL:
                        return Optional.of(new RightLeftThreeBall());
                    default:
                        return Optional.of(new OneBall());

                }

            case RIGHT_TARMAC_RIGHT:

                switch(mode) {

                    case ONE_BALL:
                        return Optional.of(new OneBall());
                    case TWO_BALL:
                        return Optional.of(new RightRightTwoBall());
                    case THREE_BALL:
                        return Optional.of(new RightRightThreeBall());
                    default:
                        return Optional.of(new OneBall());

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