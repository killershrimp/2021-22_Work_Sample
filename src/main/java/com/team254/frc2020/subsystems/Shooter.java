package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private static Shooter mInstance;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    private final TalonFX mRightMaster, mLeftMaster;

    public static class PeriodicIO {
        // inputs
        double right_velocity_ticks_per_100_ms = 0.0;
        double left_velocity_ticks_per_100_ms = 0.0;

        // outputs
        double demand = 0.0;
    }

    public enum ShooterControlState {
        OPEN_LOOP, VELOCITY
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ShooterControlState mControlState = ShooterControlState.OPEN_LOOP;

    private Shooter() {
        mRightMaster = TalonFXFactory.createDefaultTalon(Constants.kShooterRightMasterId);
        mRightMaster.setInverted(true);
        mRightMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mRightMaster.enableVoltageCompensation(true);

        mLeftMaster = TalonFXFactory.createDefaultTalon(Constants.kShooterLeftMasterId);
        mLeftMaster.setInverted(false);
        mLeftMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mLeftMaster.enableVoltageCompensation(true);

        // initialize encoders and set status frames
        TalonUtil.checkError(mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs), "Shooter Right Master: Could not detect encoder: ");
        TalonUtil.checkError(mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,
                Constants.kLongCANTimeoutMs), "Shooter Left Master: Could not detect encoder: ");

        // TODO velocity measurement windows/periods?

        // set right master gains
        TalonUtil.checkError(mRightMaster.config_kP(0, Constants.kShooterKp, Constants.kLongCANTimeoutMs),
                "Shooter Right Master: could not set kP: ");
        TalonUtil.checkError(mRightMaster.config_kI(0, Constants.kShooterKi, Constants.kLongCANTimeoutMs),
                "Shooter Right Master: could not set kI: ");
        TalonUtil.checkError(mRightMaster.config_kD(0, Constants.kShooterKd, Constants.kLongCANTimeoutMs),
                "Shooter Right Master: could not set kD: ");
        TalonUtil.checkError(mRightMaster.config_kF(0, Constants.kShooterKf, Constants.kLongCANTimeoutMs),
                "Shooter Right Master: Could not set kF: ");

        // set left master gains
        TalonUtil.checkError(mLeftMaster.config_kP(0, Constants.kShooterKp, Constants.kLongCANTimeoutMs),
                "Shooter Left Master: could not set kP: ");
        TalonUtil.checkError(mLeftMaster.config_kI(0, Constants.kShooterKi, Constants.kLongCANTimeoutMs),
                "Shooter Left Master: could not set kI: ");
        TalonUtil.checkError(mLeftMaster.config_kD(0, Constants.kShooterKd, Constants.kLongCANTimeoutMs),
                "Shooter Left Master: could not set kD: ");
        TalonUtil.checkError(mLeftMaster.config_kF(0, Constants.kShooterKf, Constants.kLongCANTimeoutMs),
                "Shooter Left Master: Could not set kF: ");
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
        mPeriodicIO.right_velocity_ticks_per_100_ms = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.left_velocity_ticks_per_100_ms = mLeftMaster.getSelectedSensorVelocity(0);
    }

    @Override
    public void writePeriodicOutputs() {
        if (mControlState == ShooterControlState.OPEN_LOOP) {
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
        } else if (mControlState == ShooterControlState.VELOCITY) {
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.demand);
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.demand);
        }
    }

    @Override
    public void stop() {
        mRightMaster.set(ControlMode.PercentOutput, 0.0);
        mLeftMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public synchronized void setOpenLoop(double power) {
        if (mControlState != ShooterControlState.OPEN_LOOP) {
            mControlState = ShooterControlState.OPEN_LOOP;
        }

        mPeriodicIO.demand = power;
    }

    public synchronized void setRPM(double rpm) {
        if (mControlState != ShooterControlState.VELOCITY) {
            mControlState = ShooterControlState.VELOCITY;
        }

        mPeriodicIO.demand = rpmToNativeUnits(rpm);
    }

    public synchronized boolean isAtSetpoint() {
        return Util.epsilonEquals(getLeftRPM(), nativeUnitsToRPM(mPeriodicIO.demand),
                Constants.kShooterAllowableErrorRPM)
                && Util.epsilonEquals(getRightRPM(), nativeUnitsToRPM(mPeriodicIO.demand),
                        Constants.kShooterAllowableErrorRPM);
    }

    public synchronized double getRightRPM() {
        return nativeUnitsToRPM(getRightVelocityNativeUnits());
    }

    public synchronized double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100_ms;
    }

    public synchronized double getLeftRPM() {
        return nativeUnitsToRPM(getLeftVelocityNativeUnits());
    }

    public synchronized double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100_ms;
    }

    public synchronized double getAverageRPM() {
        return (getRightRPM() + getLeftRPM()) / 2.0;
    }

    /**
     * @param ticks per 100 ms
     * @return rpm
     */
    public double nativeUnitsToRPM(double ticks_per_100_ms) {
        return ticks_per_100_ms * 10.0 * 60.0 / Constants.kShooterTicksPerRevolution;
    }

    /**
     * @param rpm
     * @return ticks per 100 ms
     */
    public double rpmToNativeUnits(double rpm) {
        return rpm / 60.0 / 10.0 * Constants.kShooterTicksPerRevolution;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Shooter Right Master RPM", getRightRPM());
        SmartDashboard.putNumber("Shooter Left Master RPM", getLeftRPM());
        SmartDashboard.putNumber("Shooter Demand", mControlState == ShooterControlState.OPEN_LOOP ? mPeriodicIO.demand
                : (mControlState == ShooterControlState.VELOCITY ? nativeUnitsToRPM(mPeriodicIO.demand) : 0.0));
        SmartDashboard.putBoolean("Shooter At Setpoint", isAtSetpoint());
    }
}