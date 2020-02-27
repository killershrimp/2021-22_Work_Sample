package com.team254.frc2020;

import java.util.Optional;

import com.team254.frc2020.auto.AutoModeExecutor;
import com.team254.frc2020.auto.modes.AutoModeBase;
import com.team254.frc2020.controlboard.CardinalDirection;
import com.team254.frc2020.controlboard.ControlBoard;
import com.team254.frc2020.controlboard.IControlBoard;
import com.team254.frc2020.loops.Looper;
import com.team254.frc2020.paths.TrajectoryGenerator;
import com.team254.frc2020.subsystems.*;
import com.team254.frc2020.subsystems.limelight.Limelight;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.OpenLoopCheesyDriveHelper;
import com.team254.lib.util.VelocityCheesyDriveHelper;
import com.team254.lib.wpilib.TimedRobot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private final Looper mEnabledLooper = new Looper();
    private final Looper mDisabledLooper = new Looper();

    private final IControlBoard mControlBoard = ControlBoard.getInstance();

    private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();
    private AutoModeExecutor mAutoModeExecutor;
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    // subsystems
    private final Drive mDrive = Drive.getInstance();
    private final Turret mTurret = Turret.getInstance();
    private final Limelight mLimelight = new Limelight(Constants.kLimelightConstants, Constants.kLowRes1xZoom);
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Serializer mSerializer = Serializer.getInstance();
    private final Hood mHood = Hood.getInstance();

    private final RobotState mRobotState = RobotState.getInstance();

    Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(
                RobotStateEstimator.getInstance(),
                mDrive,
                mTurret,
                mHood,
                Shooter.getInstance(),
                mSerializer,
                mIntake,
                mSuperstructure,
                mLimelight
            );

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            
            mTrajectoryGenerator.generateTrajectories();

            // Robot starts backwards, turret starts backwards (in robot frame)
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.fromRotation(Rotation2d.fromDegrees(180)), new Pose2d(Constants.kVehicleToTurretTranslation, Rotation2d.fromDegrees(Constants.kTurretConstants.kHomePosition)));
            mDrive.setHeading(Rotation2d.fromDegrees(180));

            mAutoModeSelector.updateModeCreator();

            mControlBoard.reset();

            mTurret.zeroSensors();
            mHood.resetIfAtLimit();

            SmartDashboard.putNumber("HoodAngleToSet", 50.0);

            mSubsystemManager.stop();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();

            // Reset all auto mode state.
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();

            mDrive.setBrakeMode(false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            CrashTracker.logAutoInit();

            mDisabledLooper.stop();

            // Robot starts backwards, turret starts backwards (in robot frame)
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.fromRotation(Rotation2d.fromDegrees(180)), new Pose2d(Constants.kVehicleToTurretTranslation, Rotation2d.fromDegrees(Constants.kTurretConstants.kHomePosition)));
            mDrive.setHeading(Rotation2d.fromDegrees(180));

            mAutoModeExecutor.start();

            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();

            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mSubsystemManager.stop();

            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            CrashTracker.logTestInit();
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            if (mSubsystemManager.checkSubsystems()) {
                System.out.println("ALL SYSTEMS PASSED");
            } else {
                System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            RobotState.getInstance().outputToSmartDashboard();
            mAutoModeSelector.outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }


    @Override
    public void disabledPeriodic() {
        try {
            // Update auto modes
            mAutoModeSelector.updateModeCreator();

            Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
            if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                mAutoModeExecutor.setAutoMode(autoMode.get());
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {
        try {
            mDrive.setHighGear(!mControlBoard.getWantsLowGear());
            // mDrive.setVelocity(VelocityCheesyDriveHelper.getInstance().cheesyDrive(-mControlBoard.getThrottle(),
            //         -mControlBoard.getTurn(), mControlBoard.getQuickTurn(), !mControlBoard.getWantsLowGear()));
            mDrive.setOpenLoop(OpenLoopCheesyDriveHelper.getInstance().cheesyDrive(mControlBoard.getThrottle(),
                    mControlBoard.getTurn(), mControlBoard.getQuickTurn()));
    
            if (mControlBoard.getShoot()) {
                mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
            } else if (mControlBoard.getAimCoarse() || mControlBoard.getAimFine()) {
                mSuperstructure.setShouldAimFine(mControlBoard.getAimFine());
                mSuperstructure.setWantedState(Superstructure.WantedState.AIM);
            } else if (mControlBoard.getMoveToZero()) {
                mSuperstructure.setWantedState(Superstructure.WantedState.MOVE_TO_ZERO);
            } else {
                mSuperstructure.setWantedState(Superstructure.WantedState.IDLE);
            }
    
            if (mControlBoard.getTurretHint() != CardinalDirection.NONE) {
                mSuperstructure.resetTurretJog();
                mSuperstructure.setTurretHint(mControlBoard.getTurretHint().getRotation().getDegrees());
            } else if (mControlBoard.getTurretJog() != 0.0) {
                mSuperstructure.resetTurretHint();
                mSuperstructure.setTurretJog(mControlBoard.getTurretJog() * Constants.kJogTurretScalar);
            } else {
                mSuperstructure.resetTurretHint();
                mSuperstructure.resetTurretJog();
            }
    
            // Intake
            if (mControlBoard.getDeployIntake()) {
                mIntake.deploy();
            } else if (mControlBoard.getRetractIntake()) {
                mIntake.stow();
            }

            // TODO make serializer logic smarter
            Serializer.WantedState serializer_wanted = Serializer.WantedState.IDLE;
            if (mControlBoard.getIntake()) {
                mIntake.setWantedState(Intake.WantedState.INTAKE);
                serializer_wanted = Serializer.WantedState.SERIALIZE;
            } else if (mControlBoard.getExhaust()) {
                mIntake.setWantedState(Intake.WantedState.EXHAUST);
            } else {
                mIntake.setWantedState(Intake.WantedState.IDLE);
                serializer_wanted = Serializer.WantedState.IDLE;
            }

            if (mSuperstructure.getSystemState() != Superstructure.SystemState.SHOOT) {
                mSerializer.setWantedState(serializer_wanted);
            } else {
                mSerializer.setWantedState(Serializer.WantedState.FEED);
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}
}
