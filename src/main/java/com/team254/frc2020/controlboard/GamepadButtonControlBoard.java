package com.team254.frc2020.controlboard;

import com.team254.frc2020.Constants;

public class GamepadButtonControlBoard implements IButtonControlBoard {

    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private final XboxController mController;

    private GamepadButtonControlBoard() {
        mController = new XboxController(Constants.kButtonGamepadPort);
    }

    @Override
    public boolean getShoot() {
        return mController.getTrigger(XboxController.Side.RIGHT);
    }

    @Override
    public boolean getMoveToZero() {
        return mController.getButton(XboxController.Button.BACK) && mController.getButton(XboxController.Button.START);
    }
}