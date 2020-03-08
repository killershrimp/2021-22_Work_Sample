package com.team254.frc2020.subsystems;

import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.nio.DoubleBuffer;

public class ClimbingStateMachine {
    private static final double kFeedforwardDown = 1.0 / 12.0;
    private static final int kMaxExtension = -280000;
    private static final int kClimbPosition = -100000;//-280000;
    private static final double kThrottleDeadband = 0.4;

    enum SystemState {
        PRECLIMB,
        DISENGAGING_BRAKE,
        EXTENDING,
        MANUAL,
        CLIMBING,
    }

    private Drive mDrive;

    private SystemState mSystemState = SystemState.PRECLIMB;
    private double mStateStartTime = Timer.getFPGATimestamp();
    private LatchedBoolean mDeployToggle = new LatchedBoolean();
    private LatchedBoolean mBreakToggle = new LatchedBoolean();
    private double mBreakTime = Double.NaN;

    public ClimbingStateMachine() {
        mDrive = Drive.getInstance();

    }

    public synchronized void reset() {
        mSystemState = SystemState.PRECLIMB;
        mDrive.setPTOEngaged(false);
        mDrive.setBreakEngaged(true);
        mDrive.configPTOPID(false);
        mDrive.setDeploy(false);
        mDrive.stop();
        mDeployToggle.update(true);
        mBreakToggle.update(true);
        mBreakTime = Double.NaN;
    }

    public synchronized void handle(double timestamp, double climbThrottle, boolean climb,
                                    boolean deploy, boolean breakOn, boolean breakOff) {
        double timeInState = timestamp - mStateStartTime;

        if (mDeployToggle.update(deploy)) {
           mDrive.setDeploy(!mDrive.getDeploy());
        }

        if (breakOn) {
            mDrive.setBreakEngaged(true);
            if (Double.isNaN(mBreakTime)) {
                mBreakTime = timestamp;
            }
        } else if (breakOff) {
            mDrive.setBreakEngaged(false);
            mBreakTime = Double.NaN;
        }

        boolean stopMotors = false;
        if (mDrive.getBreak() && !Double.isNaN(mBreakTime) && (timestamp - mBreakTime > 0.5)) {
            climbThrottle = 0.0;
            stopMotors = true;
        }

        climbThrottle = Util.limit(climbThrottle, 1.0);

        switch (mSystemState) {
            case PRECLIMB:
                mDrive.setPTOEngaged(true);
                mDrive.setBreakEngaged(false);
                mDrive.zeroPTOMotors();
                break;
            case DISENGAGING_BRAKE:
                // Positive throttle is downwards.
                mDrive.setPTOMotorsOpenLoop(0.1, 0.0);
            //    System.out.println("PTO position" + mDrive.getPTOPosition() + " throttle: " +
                        //climbThrottle);
                break;
            case EXTENDING:
                mDrive.setPTOMotorsPosition(kMaxExtension);
                //mDrive.setPTOMotorsOpenLoop(Util.handleDeadband(-climbThrottle, kThrottleDeadband), 0.0);
                break;
            case MANUAL:
                if (stopMotors) {
                    mDrive.setPTOMotorsOpenLoop(0.0, 0.0);
                } else if (Util.inRange(climbThrottle, kThrottleDeadband)) {
                    mDrive.setPTOMotorsPosition(mDrive.getPTOPosition());
                } else {
                    mDrive.setPTOMotorsOpenLoop(Util.handleDeadband(-climbThrottle, kThrottleDeadband), 0.0);
                }
                break;
            case CLIMBING:
                mDrive.setPTOMotorsPosition(kClimbPosition);
                break;
            default:
                break;
        }

        SystemState nextState = mSystemState;
        switch (mSystemState) {
            case PRECLIMB:
                nextState = SystemState.DISENGAGING_BRAKE;
                break;
            case DISENGAGING_BRAKE:
                if (timeInState > 0.5) {
                    nextState = SystemState.EXTENDING;
                    mDrive.setDeploy(true);
                    mDrive.configPTOPID(true);
                }
                break;
            case EXTENDING:
                if (!Util.inRange(climbThrottle, kThrottleDeadband)) {
                    nextState = SystemState.MANUAL;
                }
                if (climb) {
                    nextState = SystemState.CLIMBING;
                }
                break;
            case CLIMBING:
                if (!Util.inRange(climbThrottle, kThrottleDeadband)) {
                    nextState = SystemState.MANUAL;
                }
                break;
            case MANUAL:
                if (climb) {
                    nextState = SystemState.CLIMBING;
                }
                break;
            default:
                break;
        }

        if (nextState != mSystemState) {
            System.out.println("Transitioned from : " + mSystemState + " to " + nextState);
            mSystemState = nextState;
            mStateStartTime = timestamp;
        }

    }

}
