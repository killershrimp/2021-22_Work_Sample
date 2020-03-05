package com.team254.frc2020.subsystems;

import com.ctre.phoenix.CANifier;
import com.team254.frc2020.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    }

    private CANifier mCanifier;

    private Canifier() {
        mCanifier = new CANifier(Constants.kCanifierId);

        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.break_beam_triggered = mCanifier.getGeneralInput(CANifier.GeneralPin.LIMR);
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
}
