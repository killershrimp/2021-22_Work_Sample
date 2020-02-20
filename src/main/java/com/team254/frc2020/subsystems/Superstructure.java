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

import edu.wpi.first.wpilibj.Timer;

public class Superstructure extends Subsystem {
    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Serializer mSerializer = Serializer.getInstance();
    private Turret mTurret = Turret.getInstance();
    private Hood mHood = Hood.getInstance();
    private Shooter mShooter = Shooter.getInstance();

    private RobotState mRobotState = RobotState.getInstance();

    private Superstructure() {}

    public enum WantedState {
        IDLE,
        SHOOT,
        MOVE_TO_ZERO
    }

    public enum SystemState {
        IDLE,
        PREPARE_TO_SHOOT,
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

    private Optional<Double> turretHint = Optional.empty();

    private double mTurretFeedforwardVFromVision = 0.0;

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop(){
            @Override
            public void onStart(double timestamp) {
                mWantedState = WantedState.IDLE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    // Reset hint if needed
                    if (turretHint.isPresent() && (isOnTarget() || visionHasTarget())) {
                        turretHint = Optional.empty();
                    }

                    SystemState newState = mSystemState;
                    double timeInState = timestamp - mCurrentStateStartTime;
            
                    switch (mSystemState) {
                        case IDLE:
                            newState = handleIdle(mWantedState);
                            break;
                        case PREPARE_TO_SHOOT:
                            newState = handlePrepareToShoot(mWantedState);
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
                        mCurrentStateStartTime = Timer.getFPGATimestamp();
                        timeInState = 0.0;
                        
                    }
            
                    switch (mSystemState) {
                        case IDLE:
                            writeIdleDesiredState();
                            break;
                        case PREPARE_TO_SHOOT:
                            writePrepareToShootDesiredState(timestamp);
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
            case SHOOT:
                return SystemState.PREPARE_TO_SHOOT;
            case MOVE_TO_ZERO:
                return SystemState.MOVE_TO_ZERO;
            case IDLE:
            default:
                return SystemState.IDLE;
        }
    }

    private SystemState handlePrepareToShoot(WantedState wantedState) {
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
                return SystemState.PREPARE_TO_SHOOT;
        }
    }

    private SystemState handleShoot(WantedState wantedState) {
        switch (wantedState) {
            case MOVE_TO_ZERO:
                return SystemState.MOVE_TO_ZERO;
            case IDLE:
                return SystemState.IDLE;
            case SHOOT:
                if (!isOnTarget() || !Hood.getInstance().isAtSetpoint()) {
                    return SystemState.PREPARE_TO_SHOOT;
                }
            default:
                return SystemState.SHOOT;
        }
    }

    private SystemState handleMoveToZero(WantedState wantedState) {
        switch (wantedState) {
            case IDLE:
                return SystemState.IDLE;
            case SHOOT:
                return SystemState.PREPARE_TO_SHOOT;
            case MOVE_TO_ZERO:
            default:
                return SystemState.MOVE_TO_ZERO;
        }
    }
    
    private void writeIdleDesiredState() {
        if (turretHint.isPresent()) {
            mTurret.setPosition(turretHint.get(), 0);
        } else {
            mTurret.setOpenLoop(0.0);
        }
        mSerializer.stopRunning();
        mHood.setOpenLoop(0.0);
        mShooter.setOpenLoop(0.0);
    }

    private void writePrepareToShootDesiredState(double timestamp) {
        mSerializer.serialize();
        double visionAngle = getTurretSetpointFromVision(timestamp);
        double angleToSet = mTurret.getAngle();
        double ffToSet = 0;
        if (visionHasTarget()) {
            angleToSet = visionAngle;
            ffToSet = getTurretFeedforwardVFromVision();;
        } else if (turretHint.isPresent()) {
            angleToSet = turretHint.get();
        }
        mTurret.setPosition(angleToSet, ffToSet);
        if (mLatestAimingParameters.isPresent()) {
            mHood.setDesiredAngle(Constants.kHoodMap.getInterpolated(new InterpolatingDouble(mLatestAimingParameters.get().getRange())).value);
        }
        mShooter.setRPM(Constants.kShooterSetpointRPM);
    }



    private void writeShootDesiredState(double timestamp) {
        mSerializer.feed();
        mTurret.setPosition(getTurretSetpointFromVision(timestamp), getTurretFeedforwardVFromVision());
        if (mLatestAimingParameters.isPresent()) {
            mHood.setDesiredAngle(Constants.kHoodMap.getInterpolated(new InterpolatingDouble(mLatestAimingParameters.get().getRange())).value);
        }
        mShooter.setRPM(Constants.kShooterSetpointRPM);
    }

    private void writeMoveToZeroDesiredState() {
        mSerializer.stopRunning();
        mTurret.setMotionMagic(Constants.kTurretStartingPositionDegrees);
        mHood.setDesiredAngle(Constants.kHoodStartingPositionDegrees);
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

    public double getTurretSetpointFromVision(double timestamp) {
        mLatestAimingParameters = mRobotState.getAimingParameters(-1, Constants.kMaxGoalTrackAge);
        if (mLatestAimingParameters.isPresent()) {
            mTrackId = mLatestAimingParameters.get().getTrackId();

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

            // if (turret_setpoint < Constants.kTurretReverseSoftLimitDegrees) {
            //     turret_setpoint += 360.0;
            // }
            // if (turret_setpoint > Constants.kTurretReverseSoftLimitDegrees) {
            //     turret_setpoint -= 360.0;
            // }

            mHasTarget = true;

            if (Util.epsilonEquals(turret_error.getDegrees(), 0.0, 3.0)) {
                mOnTarget = true;
            } else {
                mOnTarget = false;
            }

            return turret_setpoint;
        } else {
            mHasTarget = false;
            mOnTarget = false;

            return mTurret.getAngle();
        }
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
        if (!visionHasTarget()) {
            turretHint = Optional.of(hint);
        }
    }
}