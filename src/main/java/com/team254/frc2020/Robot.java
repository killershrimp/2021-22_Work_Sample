package com.team254.frc2020;

import java.util.Map;
import java.util.Optional;

import com.team254.frc2020.auto.AutoModeExecutor;
import com.team254.frc2020.auto.modes.AutoModeBase;
import com.team254.frc2020.controlboard.CardinalDirection;
import com.team254.frc2020.controlboard.ControlBoard;
import com.team254.frc2020.controlboard.IControlBoard;
import com.team254.frc2020.limelight.constants.LimelightConstantsFactory;
import com.team254.frc2020.loops.Looper;
import com.team254.frc2020.paths.TrajectoryGenerator;
import com.team254.frc2020.subsystems.*;
import com.team254.frc2020.subsystems.Limelight;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.OpenLoopCheesyDriveHelper;
import com.team254.lib.util.Util;
import com.team254.lib.util.VelocityCheesyDriveHelper;
import com.team254.lib.wpilib.TimedRobot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    private final Limelight mLimelight = new Limelight(LimelightConstantsFactory.getConstantsForId(Constants.kDefaultLimelightId), Constants.kLowRes1xZoom);
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Serializer mSerializer = Serializer.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Canifier mCanifier = Canifier.getInstance();

    private Compressor mCompressor;

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
                mLimelight,
                mCanifier
            );

            mCompressor = new Compressor();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            
            mTrajectoryGenerator.generateTrajectories();

            // Robot starts backwards, turret starts backwards (in robot frame)
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.fromRotation(Rotation2d.fromDegrees(180)), new Pose2d(Constants.kVehicleToTurretTranslation, Rotation2d.fromDegrees(Constants.kTurretConstants.kHomePosition)));
            mDrive.setHeading(Rotation2d.fromDegrees(180));

            mAutoModeSelector.updateModeCreator();

            mControlBoard.reset();

            mTurret.zeroSensors();

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
            mCompressor.stop();
            // Robot starts backwards, turret starts backwards (in robot frame)
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.fromRotation(Rotation2d.fromDegrees(180)), new Pose2d(Constants.kVehicleToTurretTranslation, Rotation2d.fromDegrees(Constants.kTurretConstants.kHomePosition)));
            mDrive.setHeading(Rotation2d.fromDegrees(180));
            mEnabledLooper.start();
            mAutoModeExecutor.start();

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
            mCompressor.start();

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

            if (mDrive.checkSystem()) {
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


            if (!mHood.hasBeenZeroed()) {
                mHood.resetIfAtLimit();
            }

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

           if (mControlBoard.getZeroGyro()) {
               mDrive.setHeading(Rotation2d.fromDegrees(180));
           }

            mDrive.setHighGear(!mControlBoard.getWantsLowGear());
            // mDrive.setVelocity(VelocityCheesyDriveHelper.getInstance().cheesyDrive(mControlBoard.getThrottle(),
                    // mControlBoard.getTurn(), mControlBoard.getQuickTurn(), !mControlBoard.getWantsLowGear()));
            mDrive.setOpenLoop(OpenLoopCheesyDriveHelper.getInstance().cheesyDrive(mControlBoard.getThrottle(),
                    mControlBoard.getTurn(), mControlBoard.getQuickTurn()));

            boolean wants_aim = mControlBoard.getAimCoarse() || mControlBoard.getAimFine();

            if (mControlBoard.getShoot()) {
                mSuperstructure.setWantedState(Superstructure.WantedState.SHOOT);
            } else if (wants_aim) {
                mSuperstructure.setShootingParams(mControlBoard.getAimFine() ? Constants.kFineShootingParams : Constants.kCoarseShootingParams);
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
            if (mControlBoard.getIntake()) {
                mIntake.deploy();
            } else {
                mIntake.stow();
            }

            // TODO check getExhaust first so can exh while intake is down by pressing both r/l bumpers?
            if (Math.abs(mControlBoard.getStir()) > Constants.kSerializerStirDeadband) {
                mSerializer.setStirOverriding(true);
                mSerializer.setOpenLoop(Util.handleDeadband(mControlBoard.getStir(),
                        Constants.kSerializerStirDeadband) * Constants.kSerializerStirScalar);
            } else {
                mSerializer.setStirOverriding(false);

                if (mControlBoard.getExhaust()) {
                    mIntake.setWantedState(Intake.WantedState.EXHAUST);
                } else if (mControlBoard.getIntake()) {
                    mIntake.setWantedState(Intake.WantedState.INTAKE);
                } else {
                    mIntake.setWantedState(Intake.WantedState.IDLE);
                }
            }

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}
}
