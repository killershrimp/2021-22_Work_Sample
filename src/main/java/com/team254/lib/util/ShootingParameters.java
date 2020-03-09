package com.team254.lib.util;

import com.team254.lib.geometry.Pose2d;

public class ShootingParameters {
	private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mHoodMap;
	private final Pose2d mVisionTargetToGoalOffset;
	private final double mShooterSetpointRPM;
	private final double mShooterAllowableErrorRPM; // rpm
	private final double mTurretAllowableErrorDegrees; // °
	private final double mHoodAllowableErrorDegrees; // °

	public ShootingParameters(InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mHoodMap, Pose2d mVisionTargetToGoalOffset, double mShooterSetpointRPM, double mShooterAllowableErrorRPM, double mTurretAllowableErrorDegrees, double mHoodAllowableErrorDegrees) {
		this.mHoodMap = mHoodMap;
		this.mVisionTargetToGoalOffset = mVisionTargetToGoalOffset;
		this.mShooterSetpointRPM = mShooterSetpointRPM;
		this.mShooterAllowableErrorRPM = mShooterAllowableErrorRPM;
		this.mTurretAllowableErrorDegrees = mTurretAllowableErrorDegrees;
		this.mHoodAllowableErrorDegrees = mHoodAllowableErrorDegrees;
	}

	public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getHoodMap() {
		return mHoodMap;
	}

	public Pose2d getVisionTargetToGoalOffset() {
		return mVisionTargetToGoalOffset;
	}

	public double getShooterSetpointRPM() {
		return mShooterSetpointRPM;
	}

	public synchronized boolean isShooterAtSetpoint(double current_shooter_rpm, double shooter_setpoint) {
		return Util.epsilonEquals(current_shooter_rpm, shooter_setpoint, mShooterAllowableErrorRPM);
	}

	public synchronized boolean isTurretAtSetpoint(double current_turret_angle, double turret_setpoint) {
		return Util.epsilonEquals(current_turret_angle, turret_setpoint, mTurretAllowableErrorDegrees);
	}

	public synchronized boolean isHoodAtSetpoint(double current_hood_angle, double hood_setpoint) {
		return Util.epsilonEquals(current_hood_angle, hood_setpoint, mHoodAllowableErrorDegrees);
	}
}