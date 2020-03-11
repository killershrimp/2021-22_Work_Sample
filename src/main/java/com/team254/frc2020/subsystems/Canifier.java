package com.team254.frc2020.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
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
        boolean turret_homing_limit_switch = false;
    }

    private CANifier mCanifier;

    private Canifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 10, Constants.kLongCANTimeoutMs);

        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.break_beam_triggered = mCanifier.getGeneralInput(CANifier.GeneralPin.SDA);
        // TODO make practice bot and comp bot the same... is LIMR on practice bot, SDA on comp bot
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
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Turret Homing Limit Switch", mPeriodicIO.turret_homing_limit_switch);
    }

    public synchronized boolean isBreamBeamSensorTriggered() {
        return mPeriodicIO.break_beam_triggered;
    }

    public synchronized boolean isTurretHomed() {
        return mPeriodicIO.turret_homing_limit_switch;
    }
}
