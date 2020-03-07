package com.team254.frc2020.auto.actions;

import java.util.Optional;

import com.team254.frc2020.subsystems.Superstructure;
import com.team254.lib.geometry.Rotation2d;

public class AutoAimAction implements Action {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private Optional<Rotation2d> mHint = Optional.empty();

    public AutoAimAction() {}

    public AutoAimAction(Rotation2d hint) {
        mHint = Optional.of(hint);
    }

    @Override
    public void start() {
        if (mHint.isPresent()) {
            mSuperstructure.setTurretHint(mHint.get().getDegrees());
        }

        mSuperstructure.setWantedState(Superstructure.WantedState.AIM);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void done() {}
}