package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Serializer extends Subsystem {

    public static final double kSpinCycleDemand = 0.6;
    public static final double kRollerDemandFeed = 0.5;
    public static final double kSpinCycleOscillationTime = 0.25; // seconds before switching dir

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

    public static enum WantedState {
        IDLE,
        SERIALIZE,
        FEED
    }

    public static enum SystemState {
        IDLE,
        SERIALIZE,
        FEED
    }

    private TalonFX mSpinCycleMaster, mRightRollerMaster, mLeftRollerMaster;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;
    private double mCurrentStateStartTime = 0.0;

    private Serializer() {
        mSpinCycleMaster = TalonFXFactory.createDefaultTalon(Constants.kSerializerSpinCycleMasterId);
        mSpinCycleMaster.setInverted(true);
        mSpinCycleMaster.setNeutralMode(NeutralMode.Brake);
        mSpinCycleMaster.configOpenloopRamp(0.0);

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
            public void onStart(double timestamp) {
                synchronized (Serializer.this) {
                    mWantedState = WantedState.IDLE;
                }
            }
        
            @Override
            public void onLoop(double timestamp) {
                synchronized (Serializer.this) {
                    SystemState newState = mSystemState;
                    double timeInState = timestamp - mCurrentStateStartTime;
            
                    switch (mSystemState) {
                        case IDLE:
                            newState = handleIdle();
                            break;
                        case SERIALIZE:
                            newState = handleSerialize();
                            break;
                        case FEED:
                            newState = handleFeed();
                            break;
                        default:
                            System.out.println("unexpected serializer system state: " + mSystemState);
                            break;
                    }
            
                    if (newState != mSystemState) {
                        System.out.println(timestamp + ": Changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        mCurrentStateStartTime = timestamp;
                        timeInState = 0.0;
                    }
            
                    switch (mSystemState) {
                        case IDLE:
                            setIdleStateDemands();
                            break;
                        case SERIALIZE:
                            setSerializeStateDemands(timeInState);
                            break;
                        case FEED:
                            setFeedStateDemands();
                            break;
                        default:
                            System.out.println("Unexpected serializer system state: " + mSystemState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    private SystemState handleIdle() {
        switch (mWantedState) {
            case SERIALIZE:
                return SystemState.SERIALIZE;
            case FEED:
                return SystemState.FEED;
            case IDLE:
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleSerialize() {
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case FEED:
                return SystemState.FEED;
            case SERIALIZE:
            default:
                return SystemState.SERIALIZE;
        }
    }

    private SystemState handleFeed() {
        switch (mWantedState) {
            case IDLE:
                return SystemState.IDLE;
            case SERIALIZE:
                return SystemState.SERIALIZE;
            case FEED:
            default:
                return SystemState.FEED;
        }
    }

    private void setIdleStateDemands() {
        mPeriodicIO.spin_cycle_demand = 0.0;
        mPeriodicIO.left_roller_demand = 0.0;
        mPeriodicIO.right_roller_demand = 0.0;
    }

    private int counter = 0;
    private void setSerializeStateDemands(double timeInState) {
        counter++;
        if ((counter % 100) > 50) {
            mPeriodicIO.spin_cycle_demand = kSpinCycleDemand;
            mPeriodicIO.left_roller_demand = 0.0;
            mPeriodicIO.right_roller_demand = 0.0;
        } else {
            mPeriodicIO.spin_cycle_demand = -kSpinCycleDemand;
            mPeriodicIO.left_roller_demand = 0.0;
            mPeriodicIO.right_roller_demand = 0.0;
        }
    }

    private void setFeedStateDemands() {
        mPeriodicIO.spin_cycle_demand = kSpinCycleDemand;
        mPeriodicIO.left_roller_demand = kRollerDemandFeed;
        mPeriodicIO.right_roller_demand = -kRollerDemandFeed;
    }

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
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