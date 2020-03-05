package com.team254.frc2020;

import com.team254.frc2020.auto.modes.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum StartingPosition {
        LEFT, RIGHT, CENTER
    }

    enum DesiredMode {
        DO_NOTHING, TEST_TRAJECTORY, BLUE_WOF, Ball_10
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    public AutoModeSelector() {
        mStartPositionChooser = new SendableChooser<>();
        mStartPositionChooser.setDefaultOption("Left", StartingPosition.LEFT);
        mStartPositionChooser.addOption("Right", StartingPosition.RIGHT);
        mStartPositionChooser.addOption("Center", StartingPosition.CENTER);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test Trajectory", DesiredMode.TEST_TRAJECTORY);
        mModeChooser.addOption("Blue WOF", DesiredMode.BLUE_WOF);
        mModeChooser.addOption("10 ball thing", DesiredMode.Ball_10);
//        mModeChooser.addOption("Turn In place (180 deg)", DesiredMode.TURN_IN_PLACE);
        SmartDashboard.putData("Auto mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();

        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }

        if (startingPosition == null) {
            startingPosition = StartingPosition.LEFT;
        }

        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name()
                    + ", starting position->" + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
    }

    private boolean startingLeft(StartingPosition position) {
        return position == StartingPosition.LEFT;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode, StartingPosition position) {
        switch (mode) {
            case DO_NOTHING:
                return Optional.of(new DoNothingAutoMode());
            case TEST_TRAJECTORY:
                return Optional.of(new TestTrajectoryFollowingMode());
            case BLUE_WOF:
                return Optional.of(new BlueWOFAutoMode());
            case Ball_10:
                return Optional.of(new WOF2Mode());
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