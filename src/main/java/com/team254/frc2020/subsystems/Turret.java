package com.team254.frc2020.subsystems;

import com.team254.frc2020.Constants;

public class Turret extends ServoMotorSubsystem {
    private static Turret mInstance;

    public synchronized static Turret getInstance() {
        if (mInstance == null) {
            mInstance = new Turret(Constants.kTurretConstants);
        }

        return mInstance;
    }

    private Turret(final ServoMotorSubsystemConstants constants) {
        super(constants);

        // TODO limit switches
    }

    // Syntactic sugar.
    public synchronized double getAngle() {
        return getPosition();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
}