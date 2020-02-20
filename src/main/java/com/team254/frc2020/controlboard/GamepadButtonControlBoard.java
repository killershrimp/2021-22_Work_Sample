package com.team254.frc2020.controlboard;

import com.team254.frc2020.Constants;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.DelayedBoolean;
import edu.wpi.first.wpilibj.Timer;

public class GamepadButtonControlBoard implements IButtonControlBoard {

    private static GamepadButtonControlBoard mInstance = null;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;
    private CardinalDirection mLastCardinal;

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
    public boolean getAim() {
        return mController.getTrigger(XboxController.Side.LEFT);
    }

    @Override
    public boolean getMoveToZero() {
        return mController.getButton(XboxController.Button.BACK) && mController.getButton(XboxController.Button.START);
    }

    @Override
    public void reset() {
        mLastCardinal = CardinalDirection.NONE;
        mDPadValid = new DelayedBoolean(Timer.getFPGATimestamp(), kDPadDelay);
    }

    @Override
    public CardinalDirection getTurretHint() {
        int dPad = mController.getDPad();
        CardinalDirection newCardinal = dPad == -1 ? CardinalDirection.NONE : CardinalDirection.findClosest(Rotation2d.fromDegrees(-dPad));
        if (newCardinal != CardinalDirection.NONE && CardinalDirection.isDiagonal(newCardinal)) {
            // Latch previous direction on diagonal presses, because the D-pad sucks at diagonals.
            newCardinal = mLastCardinal;
        }
        boolean valid = mDPadValid.update(Timer.getFPGATimestamp(), newCardinal != CardinalDirection.NONE && (mLastCardinal == CardinalDirection.NONE || newCardinal == mLastCardinal));
        if (valid) {
            if (mLastCardinal == CardinalDirection.NONE) {
                mLastCardinal = newCardinal;
            }
            return mLastCardinal;
        } else {
            mLastCardinal = newCardinal;
        }
        return CardinalDirection.NONE;
    }
}