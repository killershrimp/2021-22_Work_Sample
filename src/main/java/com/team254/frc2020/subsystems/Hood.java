package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.team254.frc2020.Constants;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.Util;

public class Hood extends ServoMotorSubsystem {
    private static Hood mInstance;

    public synchronized static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood(Constants.kHoodConstants);
        }

        return mInstance;
    }

    private Hood(final ServoMotorSubsystemConstants constants) {
        super(constants);

        TalonUtil.checkError(
                mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen),
                mConstants.kName + ": Could not set reverse limit switch: ");
        mMaster.overrideLimitSwitchesEnable(true);
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    public synchronized boolean isAtSetpoint() {
        return Util.epsilonEquals(mPeriodicIO.position_ticks, mPeriodicIO.demand, mConstants.kPositionDeadband);
    }

    @Override
    public boolean atHomingLocation() {
        return mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}