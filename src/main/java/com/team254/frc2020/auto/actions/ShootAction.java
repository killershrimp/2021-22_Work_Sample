package com.team254.frc2020.auto.actions;

import com.team254.frc2020.subsystems.Serializer;
import com.team254.frc2020.subsystems.Superstructure;
import com.team254.lib.util.ShootingParameters;

import edu.wpi.first.wpilibj.Timer;

public class ShootAction implements Action {
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Serializer mSerializer = Serializer.getInstance();

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
    public void update() {
        // TODO move this somewhere better
        if (mSuperstructure.getSystemState() == Superstructure.SystemState.SHOOT) {
            mSerializer.setWantedState(Serializer.WantedState.FEED);
        }
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - mStartTime) >= mShotDuration;
    }

    @Override
    public void done() {
        mSerializer.setWantedState(Serializer.WantedState.IDLE);
        mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
    }
}