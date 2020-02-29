package com.team254.frc2020.subsystems.utils;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Talon;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicBoolean;

public class DriveWithPTOSide {
    private final TalonFX driveOnlyA, driveOnlyB, driveAndClimb;
    private Set<TalonFX> onlyDrive = new HashSet<>();
    private Set<TalonFX> allMotors = new HashSet<>();
    private Set<TalonFX> withPTO = new HashSet<>();
    private Set<TalonFX> emptySet = new HashSet<>();

    private AtomicBoolean mPtoEngaged = new AtomicBoolean(false);

    public DriveWithPTOSide(TalonFX driveAndClimb, TalonFX driveOnlyA, TalonFX driveOnlyB) {
        this.driveOnlyA = driveOnlyA;
        this.driveOnlyB = driveOnlyB;
        this.driveAndClimb = driveAndClimb;

        allMotors.add(driveAndClimb);
        allMotors.add(driveOnlyA);
        allMotors.add(driveOnlyB);

        onlyDrive.add(driveOnlyA);
        onlyDrive.add(driveOnlyB);

        withPTO.add(driveAndClimb);
    }

    public TalonFX getPrimaryDriveTalonFX() {
        return driveOnlyA;
    }

    public void setPTOEngaged(boolean engaged) {
        mPtoEngaged.set(engaged);
    }

    public Set<TalonFX> getDriveTalons() {
        if (mPtoEngaged.get()) {
            return onlyDrive;
        } else {
            return allMotors;
        }
    }

    public Set<TalonFX> getPTOMotors() {
        if (mPtoEngaged.get()) {
            return withPTO;
        } else {
            return emptySet;
        }
    }

    public Set<TalonFX> getAllMotors() {
        return allMotors;
    }
}
