package com.team254.frc2020.controlboard;

import com.team254.frc2020.Constants;

public class ControlBoard implements IControlBoard {
    private static ControlBoard mInstance = null;

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final IDriveControlBoard mDriveControlBoard;
    private final IButtonControlBoard mButtonControlBoard;

    private ControlBoard() {
        mDriveControlBoard = Constants.kUseDriveGamepad ? GamepadDriveControlBoard.getInstance()
                : MainDriveControlBoard.getInstance();
        mButtonControlBoard = GamepadButtonControlBoard.getInstance();
    }

    @Override
    public double getThrottle() {
        return mDriveControlBoard.getThrottle();
    }

    @Override
    public double getTurn() {
        return mDriveControlBoard.getTurn();
    }

    @Override
    public boolean getQuickTurn() {
        return mDriveControlBoard.getQuickTurn();
    }

    @Override
    public boolean getWantsLowGear() {
        return mDriveControlBoard.getWantsLowGear();
    }

    @Override
    public boolean getShoot() {
        return mButtonControlBoard.getShoot();
    }

    @Override
    public boolean getMoveToZero() {
        return mButtonControlBoard.getMoveToZero();
    }
}