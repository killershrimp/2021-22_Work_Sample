package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team254.frc2020.Constants;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.drivers.TalonFXFactory;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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
        mMaster.setInverted(true);

        mDeploySolenoid = new Solenoid(Constants.kPCMId, Constants.kIntakeSolenoidId);
        mPeriodicIO = new PeriodicIO();
    }

    public enum WantedState {
        IDLE,
        INTAKE,
        EXHAUST
    }

    public enum SystemState {
        IDLE,
        INTAKING,
        EXHAUSTING,
        LIGHT_INTAKE, // small intake amt after stowing intake automatically
    }

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;
    private double mCurrentStateStartTime = 0.0;
    private double mLastStowTime = 0.0;

    private static class PeriodicIO {
        double motor_demand = 0.0;
    }

    public synchronized void deploy() {
        mDeploySolenoid.set(kIntakeDeployVal);
        mLastStowTime = 0.0;
        isStowed = false;
    }

    private boolean isStowed = false;
    public synchronized boolean isStowed() {
        return isStowed;
    }


    public synchronized void stow() {
        stop();
        mDeploySolenoid.set(!kIntakeDeployVal);
        if (!isStowed) {
            mLastStowTime = Timer.getFPGATimestamp();
        }
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
                synchronized (Intake.this) {
                    mWantedState = WantedState.IDLE;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                    SystemState newState = mSystemState;
                    double timeInState = timestamp - mCurrentStateStartTime;

                    switch (mSystemState) {
                        case IDLE:
                        case INTAKING:
                        case EXHAUSTING:
                        case LIGHT_INTAKE:
                            newState = handleAll(timestamp);
                            break;
                        default:
                            System.out.println("unexpected serializer system state: " + mSystemState);
                            break;
                    }

                    if (newState != mSystemState) {
                        System.out.println(timestamp + ": Intake changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        mCurrentStateStartTime = timestamp;
                        timeInState = 0.0;
                    }

                    switch (mSystemState) {
                        case IDLE:
                            mPeriodicIO.motor_demand = 0;
                            break;
                        case INTAKING:
                            mPeriodicIO.motor_demand = Constants.kIntakePower;
                            break;
                        case EXHAUSTING:
                            mPeriodicIO.motor_demand = Constants.kIntakeExhaustPower;
                            break;
                        case LIGHT_INTAKE:
                            mPeriodicIO.motor_demand = Constants.kLightIntakePower;
                            break;
                        default:
                            System.out.println("Unexpected intake system state: " + mSystemState);
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

    private SystemState handleAll(double timestamp) {
        double timeSinceStowCommand = timestamp - mLastStowTime;
        switch (mWantedState) {
            case INTAKE:
                if (timeSinceStowCommand < Constants.kIntakeStowTime) {
                    return SystemState.IDLE;
                }
                return SystemState.INTAKING;
            case EXHAUST:
                return SystemState.EXHAUSTING;
            case IDLE:
                if (timeSinceStowCommand > Constants.kIntakeStowTime && (timeSinceStowCommand - Constants.kIntakeStowTime) < Constants.kLightIntakeTime) {
                    return SystemState.LIGHT_INTAKE;
                }
                return SystemState.IDLE;
            default:
                return SystemState.IDLE;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.motor_demand);
    }

    @Override
    public void stop() {
        mPeriodicIO.motor_demand = 0.0;
        mMaster.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    public void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Intake motor demand:", mPeriodicIO.motor_demand);
        SmartDashboard.putBoolean("Intake is stowed", isStowed());
    }
}
