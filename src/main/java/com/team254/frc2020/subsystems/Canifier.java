package com.team254.frc2020.subsystems;

import com.ctre.phoenix.CANifier;
import com.team254.frc2020.Constants;

public class Canifier extends Subsystem {

    private static Canifier mInstance;

    public synchronized static Canifier getInstance() {
        if (mInstance == null) {
            mInstance = new Canifier();
        }
        return mInstance;
    }

    private PeriodicIO mPeriodicIO;

    public static class PeriodicIO {
        boolean break_beam_triggered = false;
        boolean turret_homing_limit_switch = false;
    }

    private CANifier mCanifier;

    private Canifier() {
        mCanifier = new CANifier(Constants.kCanifierId);

        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.break_beam_triggered = mCanifier.getGeneralInput(CANifier.GeneralPin.LIMR);
        mPeriodicIO.turret_homing_limit_switch = !mCanifier.getGeneralInput(CANifier.GeneralPin.LIMF);
    }

    @Override
    public void stop() {
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}

    public synchronized boolean isBreamBeamSensorTriggered() {
        return mPeriodicIO.break_beam_triggered;
    }

    public synchronized boolean isTurretHomed() {
        return mPeriodicIO.turret_homing_limit_switch;
    }
}
