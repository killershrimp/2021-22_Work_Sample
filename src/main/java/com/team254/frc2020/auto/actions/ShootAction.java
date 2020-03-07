package com.team254.frc2020.auto.actions;

import com.team254.frc2020.subsystems.Superstructure;
import com.team254.lib.util.ShootingParameters;

import edu.wpi.first.wpilibj.Timer;

public class ShootAction implements Action {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    private final ShootingParameters mShootingParameters;

    private double mShotDuration;
    private double mStartTime = 0.0;

    public ShootAction(ShootingParameters params, double duration) {
        mShootingParameters = params;
        mShotDuration = duration;
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();

        mSuperstructure.setShootingParams(mShootingParameters);
        mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - mStartTime) >= mShotDuration;
    }

    @Override
    public void done() {
        mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
    }
}