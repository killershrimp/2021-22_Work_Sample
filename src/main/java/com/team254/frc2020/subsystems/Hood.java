package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hood extends Subsystem {

    private static Hood mInstance;

    public static Hood getInstance() {
        if (mInstance == null) {
            mInstance = new Hood();
        }

        return mInstance;
    }

    private TalonFX mMaster;

    public static class PeriodicIO {
        // inputs
        double ticks = 0.0; // ticks
        boolean limit_switch = false;

        // outputs
        double demand = 0.0; // ticks
    }

    public enum HoodControlMode {
        OPEN_LOOP, POSITION
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private HoodControlMode mControlMode = HoodControlMode.OPEN_LOOP;

    private Hood() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kHoodMasterId);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.setInverted(true);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        TalonUtil.checkError(mMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen), "Could not config reverse limit switch hood");
        mMaster.overrideLimitSwitchesEnable(true);
        // initialize encoder and set status frame
        TalonUtil.checkError(
                mMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, Constants.kLongCANTimeoutMs),
                "Hood Master: Could not detect encoder: ");
        TalonUtil.checkError(
                mMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, Constants.kLongCANTimeoutMs),
                "Hood Master: Could not set hood feedback frame: ");

        // set master gains
        TalonUtil.checkError(mMaster.config_kP(0, Constants.kHoodKp, Constants.kLongCANTimeoutMs),
                "Hood Master: could not set kP: ");
        TalonUtil.checkError(mMaster.config_kI(0, Constants.kHoodKi, Constants.kLongCANTimeoutMs),
                "Hood Master: could not set kI: ");
        TalonUtil.checkError(mMaster.config_kD(0, Constants.kHoodKd, Constants.kLongCANTimeoutMs),
                "Hood Master: could not set kD: ");
        TalonUtil.checkError(mMaster.config_kF(0, Constants.kHoodKf, Constants.kLongCANTimeoutMs),
                "Hood Master: Could not set kF: ");
        TalonUtil.checkError(mMaster.configAllowableClosedloopError(0, Constants.kHoodAllowableClosedloopError,
                Constants.kLongCANTimeoutMs), "Hood Master: Could not set allowable closed loop error: ");

        // soft limits
        TalonUtil.checkError(
                        mMaster.configForwardSoftLimitThreshold(degreesToTicks(Constants.kHoodForwardSoftLimitDegrees),
                                Constants.kLongCANTimeoutMs),
                        "Hood Master: Could not set forward soft limit threshold: ");
        TalonUtil.checkError(
                        mMaster.configReverseSoftLimitThreshold(degreesToTicks(Constants.kHoodReverseSoftLimitDegrees),
                                Constants.kLongCANTimeoutMs),
                        "Hood Master: Could not set reverse soft limit threshold: ");
        TalonUtil.checkError(mMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                "Hood Master: Could not enable forward soft limit: ");
        TalonUtil.checkError(mMaster.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                "Hood Master: Could not enable reverse soft limit: ");
        mMaster.overrideSoftLimitsEnable(true);

        mMaster.configGetStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 5, 5, 0.2));

        zeroSensors();

    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.ticks = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.limit_switch = mMaster.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    @Override
    public void writePeriodicOutputs() {
        switch (mControlMode) {
        case OPEN_LOOP:
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
            break;
        case POSITION:
            mMaster.set(ControlMode.Position, mPeriodicIO.demand);
            break;
        default:
            mMaster.set(ControlMode.PercentOutput, 0.0);
            break;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void setOpenLoop(double demand) {
        mControlMode = HoodControlMode.OPEN_LOOP;
        if (Math.abs(demand) > .05) {
            mPeriodicIO.demand = demand;
        } else {
            mPeriodicIO.demand = 0;
        }
    }

    public synchronized void setDesiredAngle(double angle) {
        mControlMode = HoodControlMode.POSITION;
        double boundedAngle = Util.limit(angle, Constants.kHoodReverseSoftLimitDegrees,
                Constants.kHoodForwardSoftLimitDegrees);
        mPeriodicIO.demand = degreesToTicks(boundedAngle);
    }

    public synchronized boolean isAtSetpoint() {
        return Util.epsilonEquals(mPeriodicIO.ticks, mPeriodicIO.demand, Constants.kHoodAllowableClosedloopError);
    }

    public synchronized double getAngle() {
        return ticksToDegrees(mPeriodicIO.ticks);
    }

    public double ticksToDegrees(double ticks) {
        return ticks * Constants.kHoodDegreesPerTick;
    }

    public int degreesToTicks(double degrees) {
        return (int) (degrees / Constants.kHoodDegreesPerTick);
    }

    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(degreesToTicks(Constants.kHoodStartingPositionDegrees), 0,
                Constants.kCANTimeoutMs);
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Hood Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Hood Angle", getAngle());
        SmartDashboard.putBoolean("Hood At Setpoint", isAtSetpoint());
        SmartDashboard.putBoolean("Hood Homed", mPeriodicIO.limit_switch);
    }
}