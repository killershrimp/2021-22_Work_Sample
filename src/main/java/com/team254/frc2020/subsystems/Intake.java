package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends Subsystem {
    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    private static final boolean kIntakeDeployVal = true;

    private TalonFX mMaster;
    private Solenoid mDeploySolenoid;
    private PeriodicIO mPeriodicIO;

    private Intake() {
        mMaster = TalonFXFactory.createDefaultTalon(Constants.kIntakeMasterId);
        mDeploySolenoid = new Solenoid(Constants.kPCMId, Constants.kIntakeSolenoidId);
        mPeriodicIO = new PeriodicIO();
    }

    private class PeriodicIO {
        double motor_demand = 0.0;
    }

    public synchronized void deploy() {
        mDeploySolenoid.set(kIntakeDeployVal);
        isStowed = false;
    }

    private boolean isStowed = false;
    public synchronized boolean isStowed() {
        return isStowed;
    }


    public synchronized void stow() {
        stop();
        mDeploySolenoid.set(!kIntakeDeployVal);
        isStowed = true;
    }

    public synchronized void setOpenLoop(double demand) {
        mPeriodicIO.motor_demand = demand;
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.motor_demand);
    }

    @Override
    public void stop() {
        mPeriodicIO.motor_demand = 0.0;
//        mMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake motor demand:", mPeriodicIO.motor_demand);
        SmartDashboard.putBoolean("Intake is stowed", isStowed());
    }
}
