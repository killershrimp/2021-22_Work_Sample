package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret extends Subsystem {
    private static Turret mInstance;

    public static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret();
        }

        return mInstance;
    }

    private final TalonFX mMaster;

    public static class PeriodicIO {
        // inputs
        double ticks = 0.0;
        
        // outputs
        double demand = 0.0;
        double feedforward = 0.0;
    }

    public enum TurretControlState {
        OPEN_LOOP,
        POSITION
    }

    private static final int kPositionProfileSlot = 0;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private TurretControlState mControlState = TurretControlState.OPEN_LOOP;

    private Turret() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kTurretMasterId);
        mMaster.setInverted(false);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mMaster.enableVoltageCompensation(true);

        // initialize encoder and set status frame
        TalonUtil.checkError(mMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs), "Turret Master: Could not detect encoder: ");
        TalonUtil.checkError(mMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, Constants.kLongCANTimeoutMs), "Turret Master: Could not set turret feedback frame: ");

        // set master gains for position
        TalonUtil.checkError(mMaster.config_kP(kPositionProfileSlot, Constants.kTurretPositionKp, Constants.kLongCANTimeoutMs),
            "Turret Master: could not set position kP: ");
        TalonUtil.checkError(mMaster.config_kI(kPositionProfileSlot, Constants.kTurretPositionKi, Constants.kLongCANTimeoutMs),
            "Turret Master: could not set position kI: ");
        TalonUtil.checkError(mMaster.config_kD(kPositionProfileSlot, Constants.kTurretPositionKd, Constants.kLongCANTimeoutMs),
            "Turret Master: could not set position kD: ");
        TalonUtil.checkError(mMaster.config_kF(kPositionProfileSlot, Constants.kTurretPositionKf, Constants.kLongCANTimeoutMs),
            "Turret Master: Could not set position kF: ");
        TalonUtil.checkError(mMaster.configAllowableClosedloopError(kPositionProfileSlot, Constants.kTurretPositionAllowableClosedloopError, Constants.kLongCANTimeoutMs),
            "Turret Master: Could not set position allowable closed loop error: ");

        // soft limits
        TalonUtil.checkError(
                mMaster.configForwardSoftLimitThreshold((int) degreesToTicks(Constants.kTurretForwardSoftLimitDegrees), Constants.kLongCANTimeoutMs),
                "Turret Master: Could not set forward soft limit: ");
        TalonUtil.checkError(mMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                "Turret Master: Could not enable forward soft limit: ");

        TalonUtil.checkError(
                mMaster.configReverseSoftLimitThreshold((int) degreesToTicks(Constants.kTurretReverseSoftLimitDegrees), Constants.kLongCANTimeoutMs),
                "Turret Master: Could not set reverse soft limit: ");
        TalonUtil.checkError(mMaster.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                "Turret Master: Could not enable reverse soft limit: ");
        mMaster.overrideSoftLimitsEnable(true);

        zeroSensors();
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

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.ticks = mMaster.getSelectedSensorPosition(0);
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == TurretControlState.OPEN_LOOP) {
            mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        } else if (mControlState == TurretControlState.POSITION) {
            mMaster.set(ControlMode.Position, mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
        }
    }

    @Override
    public void zeroSensors() {
        mMaster.setSelectedSensorPosition((int) degreesToTicks(Constants.kTurretStartingPositionDegrees));
    }

    @Override
    public void stop() {
        mMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public synchronized void setOpenLoop(double power) {
        if (mControlState != TurretControlState.OPEN_LOOP) {
            mControlState = TurretControlState.OPEN_LOOP;
        }

        mPeriodicIO.demand = power;
    }

    /**
     * @param angle in degrees
     */
    public synchronized void setPosition(double degrees) {
        setPosition(degrees, 0.0);
    }

    /**
     * @param angle in degrees
     * @param feedforward_v
     */
    public synchronized void setPosition(double degrees, double feedforward_v) {
        if (mControlState != TurretControlState.POSITION) {
            mControlState = TurretControlState.POSITION;
            mMaster.selectProfileSlot(kPositionProfileSlot, 0);
        }

        mPeriodicIO.demand = degreesToTicks(degrees);
        double feedforward_ticks_per_100ms = degreesToTicks(feedforward_v) / 10.0; // deg / s to ticks / 100 ms
        mPeriodicIO.feedforward = 0.0 * feedforward_ticks_per_100ms;  // TODO tune
    }

    public synchronized double getAngle() {
        return ticksToDegrees(getTicks());
    }

    public synchronized double getTicks() {
        return mPeriodicIO.ticks;
    }

    /**
     * @param ticks
     * @return angle in degrees
     */
    public double ticksToDegrees(double ticks) {
        return ticks * Constants.kTurretDegreesPerTick;
    }

    /**
     * @param angle in degrees
     * @return ticks
     */
    public double degreesToTicks(double degrees) {
        return degrees / Constants.kTurretDegreesPerTick;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Turret Ticks", getTicks());
        SmartDashboard.putNumber("Turret Angle (°)", getAngle());
        SmartDashboard.putNumber("Turret Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("Turret Demand (°)", ticksToDegrees(mPeriodicIO.demand));
    }
}