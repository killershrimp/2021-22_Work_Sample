package com.team254.lib.util;

import com.team254.lib.geometry.Pose2d;

public class ShootingParameters {
	private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mHoodMap;
	private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterRPMMap;
	private final Pose2d mVisionTargetToGoalOffset;
	private final double mSpinCycleSetpoint; // percent output
	private final double mShooterAllowableErrorRPM; // rpm
	private final double mTurretAllowableErrorDegrees; // °
	private final double mHoodAllowableErrorDegrees; // °

	public ShootingParameters(InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodMap,
			InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shooterRPMMap,
			Pose2d visionTargetToGoalOffset,
			double spinCycleSetpoint,
			double shooterAllowableErrorRPM,
			double turretAllowableErrorDegrees,
			double hoodAllowableErrorDegrees) {
		this.mHoodMap = hoodMap;
		this.mShooterRPMMap = shooterRPMMap;
		this.mVisionTargetToGoalOffset = visionTargetToGoalOffset;
		this.mSpinCycleSetpoint = spinCycleSetpoint;
		this.mShooterAllowableErrorRPM = shooterAllowableErrorRPM;
		this.mTurretAllowableErrorDegrees = turretAllowableErrorDegrees;
		this.mHoodAllowableErrorDegrees = hoodAllowableErrorDegrees;
	}

	public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getHoodMap() {
		return mHoodMap;
	}

	public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterRPMMap() {
		return mShooterRPMMap;
	}

	public Pose2d getVisionTargetToGoalOffset() {
		return mVisionTargetToGoalOffset;
	}

	public double getSpinCycleSetpoint() {
		return mSpinCycleSetpoint;
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