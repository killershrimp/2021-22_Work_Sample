package com.team254.frc2020.subsystems;

import java.util.Optional;

import com.team254.frc2020.Constants;
import com.team254.frc2020.RobotState;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Units;
import com.team254.lib.util.Util;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Superstructure extends Subsystem {
    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Turret mTurret = Turret.getInstance();
    private Hood mHood = Hood.getInstance();
    private Shooter mShooter = Shooter.getInstance();

    private RobotState mRobotState = RobotState.getInstance();

    private Superstructure() {}

    public enum WantedState {
        IDLE,
        AIM,
        SHOOT,
        MOVE_TO_ZERO
    }

    public enum SystemState {
        IDLE,
        AIMING,
        SHOOT,
        MOVE_TO_ZERO
    }

    private WantedState mWantedState = WantedState.IDLE;
    private SystemState mSystemState = SystemState.IDLE;
    private double mCurrentStateStartTime = 0.0;

    private boolean mHasTarget = false;
    private boolean mOnTarget = false;
    private int mTrackId = -1;

    private Optional<AimingParameters> mLatestAimingParameters = Optional.empty();
    private double mCorrectedRangeToTarget = 0.0;
    private boolean mEnforceAutoAimMinDistance = false;
    private double mAutoAimMinDistance = 500;

    private Optional<Double> mTurretHint = Optional.empty();
    private Optional<Double> mTurretJogDelta = Optional.empty();
    private boolean mShouldAimFine = false; // fine aim is 3 pt, coarse aim is 2 pt

    private double mTurretFeedforwardVFromVision = 0.0;

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop(){
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                    mWantedState = WantedState.IDLE;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    SystemState newState = mSystemState;
                    double timeInState = timestamp - mCurrentStateStartTime;
            
                    switch (mSystemState) {
                        case IDLE:
                            newState = handleIdle(mWantedState);
                            break;
                        case AIMING:
                            newState = handleAiming(mWantedState);
                            break;
                        case SHOOT:
                            newState = handleShoot(mWantedState);
                            break;
                        case MOVE_TO_ZERO:
                            newState = handleMoveToZero(mWantedState);
                            break;
                        default:
                            System.out.println("unexpected superstructure system state: " + mSystemState);
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
                            writeIdleDesiredState(timestamp);
                            break;
                        case AIMING:
                            writeAimingDesiredState(timestamp);
                            break;
                        case SHOOT:
                            writeShootDesiredState(timestamp);
                            break;
                        case MOVE_TO_ZERO:
                            writeMoveToZeroDesiredState();
                            break;
                        default:
                            System.out.println("Unexpected superstructure system state: " + mSystemState);
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

    private SystemState handleIdle(WantedState wantedState) {
        switch (wantedState) {
            case AIM:
                return SystemState.AIMING;
            case MOVE_TO_ZERO:
                return SystemState.MOVE_TO_ZERO;
            case IDLE:
            case SHOOT:
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handleAiming(WantedState wantedState) {
        switch (wantedState) {
            case MOVE_TO_ZERO:
                return SystemState.MOVE_TO_ZERO;
            case IDLE:
                return SystemState.IDLE;
            case SHOOT:
                if (isOnTarget() && Shooter.getInstance().isAtSetpoint() && Hood.getInstance().isAtSetpoint()) {
                    return SystemState.SHOOT;
                }
            default:
                return SystemState.AIMING;
        }
    }

    private SystemState handleShoot(WantedState wantedState) {
        switch (wantedState) {
            case MOVE_TO_ZERO:
                return SystemState.MOVE_TO_ZERO;
            case IDLE:
                return SystemState.IDLE;
            case AIM:
                return SystemState.AIMING;
            case SHOOT:
                if (!isOnTarget() || !Hood.getInstance().isAtSetpoint()) {
                    return SystemState.AIMING;
                }
            default:
                return SystemState.SHOOT;
        }
    }

    private SystemState handleMoveToZero(WantedState wantedState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case AIM:
                return SystemState.AIMING;
            case SHOOT:
                return SystemState.AIMING;
            case MOVE_TO_ZERO:
            default:
                return SystemState.MOVE_TO_ZERO;
        }
    }
    
    private void writeIdleDesiredState(double timestamp) {
        if (mTurretHint.isPresent()) {
            mTurret.setSetpointPositionPID(getTurretSetpointFromFieldRelativeGoal(timestamp, mTurretHint.get()));
        } else if (mTurretJogDelta.isPresent()) {
            mTurret.setSetpointPositionPID(mTurret.getAngle() + mTurretJogDelta.get());
        } else {
            mTurret.setSetpointPositionPID(mTurret.getAngle());
        }
        mHood.setSetpointPositionPID(mHood.getAngle());
        mShooter.setOpenLoop(0.0);
    }

    private void writeAimingDesiredState(double timestamp) {
        double visionAngle = getTurretSetpointFromVision(timestamp);
        double angleToSet = mTurret.getAngle();
        double ffToSet = 0;
        if (visionHasTarget()) {
            angleToSet = visionAngle;
            ffToSet = getTurretFeedforwardVFromVision();
        } else if (mTurretHint.isPresent()) {
            angleToSet = getTurretSetpointFromFieldRelativeGoal(timestamp, mTurretHint.get());
        } else if (mTurretJogDelta.isPresent()) {
            mTurret.setSetpointPositionPID(mTurret.getAngle() + mTurretJogDelta.get());
        }
        mTurret.setSetpointPositionPID(angleToSet, ffToSet);

        // TODO uncomment for actual use
        // START
        // if (mLatestAimingParameters.isPresent()) {
        //     double setpoint;
        //     if (mShouldAimFine) {
        //         setpoint = Constants.kFineHoodMap.getInterpolated(new InterpolatingDouble(mLatestAimingParameters.get().getRange())).value;
        //     } else {
        //         setpoint = Constants.kCoarseHoodMap.getInterpolated(new InterpolatingDouble(mLatestAimingParameters.get().getRange())).value;
        //     }

        //     mHood.setSetpointPositionPID(setpoint);
        // }
        // END

        // TODO uncomment for tuning
        // START
        mHood.setSetpointPositionPID(SmartDashboard.getNumber("HoodAngleToSet", 50.0));
        // END

        mShooter.setRPM(Constants.kShooterSetpointRPM);
    }

    private void writeShootDesiredState(double timestamp) {
        mTurret.setSetpointPositionPID(getTurretSetpointFromVision(timestamp), getTurretFeedforwardVFromVision());
        
        // TODO uncomment for actual use
        // START
        // if (mLatestAimingParameters.isPresent()) {
        //     double setpoint;
        //     if (mShouldAimFine) {
        //         setpoint = Constants.kFineHoodMap.getInterpolated(new InterpolatingDouble(mLatestAimingParameters.get().getRange())).value;
        //     } else {
        //         setpoint = Constants.kCoarseHoodMap.getInterpolated(new InterpolatingDouble(mLatestAimingParameters.get().getRange())).value;
        //     }

        //     mHood.setSetpointPositionPID(setpoint);
        // }

        // TODO uncomment for tuning
        // START
        mHood.setSetpointPositionPID(SmartDashboard.getNumber("HoodAngleToSet", 50.0));
        // END

        mShooter.setRPM(Constants.kShooterSetpointRPM);
    }

    private void writeMoveToZeroDesiredState() {
        mTurret.setSetpointPositionPID(Constants.kTurretConstants.kHomePosition);
        mHood.setSetpointPositionPID(Constants.kHoodConstants.kHomePosition);
        mShooter.setOpenLoop(0.0);
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {}

    @Override
    public void writePeriodicOutputs() {}

    public synchronized void setWantedState(WantedState wantedState) {
        mWantedState = wantedState;
    }

    public synchronized WantedState getWantedState() {
        return mWantedState;
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public boolean visionHasTarget() {
        return mHasTarget;
    }

    public synchronized double getTurretSetpointFromVision(double timestamp) {
        mLatestAimingParameters = mRobotState.getAimingParameters(-1, Constants.kMaxGoalTrackAge, mShouldAimFine);
        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

            SmartDashboard.putNumber("Range To Target", mLatestAimingParameters.get().getRange());

            final double kLookaheadTime = 0.7;
            Pose2d robot_to_predicted_robot = mRobotState.getLatestFieldToVehicle().getValue().inverse()
                    .transformBy(mRobotState.getPredictedFieldToVehicle(kLookaheadTime));
            Pose2d predicted_robot_to_goal = robot_to_predicted_robot.inverse()
                    .transformBy(mLatestAimingParameters.get().getRobotToGoal());
            mCorrectedRangeToTarget = predicted_robot_to_goal.getTranslation().norm();

            // Don't aim if not in min distance
            if (mEnforceAutoAimMinDistance && mCorrectedRangeToTarget > mAutoAimMinDistance) {
                return mTurret.getAngle();
            }

            Rotation2d turret_error = mRobotState.getVehicleToTurret(timestamp).getRotation().inverse()
                    .rotateBy(mLatestAimingParameters.get().getRobotToGoalRotation());
            double turret_setpoint = mTurret.getAngle() + turret_error.getDegrees();
            Twist2d velocity = mRobotState.getMeasuredVelocity();
            // Angular velocity component from tangential robot motion about the goal.
            double tangential_component = mLatestAimingParameters.get().getRobotToGoalRotation().sin() * velocity.dx / mLatestAimingParameters.get().getRange();
            double angular_component = Units.radians_to_degrees(velocity.dtheta);
            // Add (opposite) of tangential velocity about goal + angular velocity in local frame.
            mTurretFeedforwardVFromVision = -(angular_component + tangential_component);

            mHasTarget = true;

            if (Util.epsilonEquals(turret_error.getDegrees(), 0.0, 3.0)) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }

            return Util.limitTurret(turret_setpoint);
        } else {
            mHasTarget = false;
            mOnTarget = false;

            return mTurret.getAngle();
        }
    }

    /**
     * @param field_relative_goal in degrees
     */
    public synchronized double getTurretSetpointFromFieldRelativeGoal(double timestamp, double field_relative_goal) {
        final double kLookaheadTime = 0.7;
        Rotation2d turret_error = mRobotState.getPredictedFieldToVehicle(kLookaheadTime)
                .transformBy(mRobotState.getVehicleToTurret(timestamp)).getRotation().inverse()
                .rotateBy(Rotation2d.fromDegrees(field_relative_goal));
        double turret_setpoint = mTurret.getAngle() + turret_error.getDegrees();

        return Util.limitTurret(turret_setpoint);
    }

    /**
     * pre condition: getTurretSetpointFromVision() is called
     * @return turret feedforward voltage
     */
    public synchronized double getTurretFeedforwardVFromVision() {
        return mTurretFeedforwardVFromVision;
    }

    public synchronized void resetAimingParameters() {
        mHasTarget = false;
        mOnTarget = false;
        mTurretFeedforwardVFromVision = 0.0;
        mTrackId = -1;
        mLatestAimingParameters = Optional.empty();
    }

    public synchronized boolean isOnTarget() {
        return mOnTarget;
    }

    public synchronized void setTurretHint(double hint) {
        mTurretHint = Optional.of(hint);
    }

    public synchronized void resetTurretHint() {
        mTurretHint = Optional.empty();
    }

    public synchronized void setTurretJog(double jog_delta) {
        mTurretJogDelta = Optional.of(jog_delta);
    }

    public synchronized void resetTurretJog() {
        mTurretJogDelta = Optional.empty();
    }

    public synchronized void setShouldAimFine(boolean should_aim_fine) {
        mShouldAimFine = should_aim_fine;
    }
}