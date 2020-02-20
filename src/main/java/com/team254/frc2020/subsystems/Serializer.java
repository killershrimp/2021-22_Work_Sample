package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Serializer extends Subsystem {

    public static final double kSpinCycleDemand = 0.4;
    public static final double kLeftRollerDemand = 1.0;
    public static final double kRightRollerDemand = 1.0;

    private static Serializer mInstance;

    public static Serializer getInstance() {
        if (mInstance == null) {
            mInstance = new Serializer();
        }

        return mInstance;
    }

    public static class PeriodicIO {
        // outputs
        double spin_cycle_demand = 0.0;
        double right_roller_demand = 0.0;
        double left_roller_demand = 0.0;
    }

    private TalonFX mSpinCycleMaster, mRightRollerMaster, mLeftRollerMaster;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private Serializer() {
        mSpinCycleMaster = TalonFXFactory.createDefaultTalon(Constants.kSerializerSpinCycleMasterId);
        mSpinCycleMaster.setInverted(true);
        mSpinCycleMaster.configOpenloopRamp(0.5);

        mSpinCycleMaster.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 50, 50, 0.2), Constants.kLongCANTimeoutMs);

        mRightRollerMaster = TalonFXFactory.createDefaultTalon(Constants.kSerializerRightRollerMasterId);
        mRightRollerMaster.setInverted(false);

        mLeftRollerMaster = TalonFXFactory.createDefaultTalon(Constants.kSerializerLeftRollerMasterId);
        mLeftRollerMaster.setInverted(false);

        // TODO 
    }

    @Override
    public void writePeriodicOutputs() {
        mSpinCycleMaster.set(ControlMode.PercentOutput, mPeriodicIO.spin_cycle_demand);
        mRightRollerMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_roller_demand);
        mLeftRollerMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_roller_demand);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop(){        
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

    public synchronized void stopRunning() {
        mPeriodicIO.spin_cycle_demand = 0.0;
        mPeriodicIO.right_roller_demand = 0.0;
        mPeriodicIO.left_roller_demand = 0.0;
    }

    public synchronized void serialize() {
        mPeriodicIO.spin_cycle_demand = kSpinCycleDemand;
        mPeriodicIO.right_roller_demand = kRightRollerDemand;
        mPeriodicIO.left_roller_demand = kLeftRollerDemand;

        mRightRollerMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 5, 5, 0.2), Constants.kCANTimeoutMs);
        mLeftRollerMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 5, 5, 0.2), Constants.kCANTimeoutMs);
    }

    public synchronized void reverseSerialize() {
        mPeriodicIO.spin_cycle_demand = -kSpinCycleDemand;
        mPeriodicIO.right_roller_demand = -kRightRollerDemand;
        mPeriodicIO.left_roller_demand = -kLeftRollerDemand;

        mRightRollerMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 5, 5, 0.2), Constants.kCANTimeoutMs);
        mLeftRollerMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 5, 5, 0.2), Constants.kCANTimeoutMs);
    }

    public synchronized void feed() {
        mPeriodicIO.spin_cycle_demand = kSpinCycleDemand;
        mPeriodicIO.right_roller_demand = -kRightRollerDemand;
        mPeriodicIO.left_roller_demand = kLeftRollerDemand;

        mRightRollerMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 10, 10, 0.2), Constants.kCANTimeoutMs);
        mLeftRollerMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 10, 10, 0.2), Constants.kCANTimeoutMs);
    }

    @Override
    public void stop() {
        mSpinCycleMaster.set(ControlMode.PercentOutput, 0.0);
        mRightRollerMaster.set(ControlMode.PercentOutput, 0.0);
        mLeftRollerMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Serializer Spin Cycle Demand", mPeriodicIO.spin_cycle_demand);
        SmartDashboard.putNumber("Serializer Right Roller Demand", mPeriodicIO.right_roller_demand);
        SmartDashboard.putNumber("Serializer Left Roller Demand", mPeriodicIO.left_roller_demand);
    }
}