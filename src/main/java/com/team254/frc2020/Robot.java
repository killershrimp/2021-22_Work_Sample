package com.team254.frc2020;

import java.sql.Time;
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
import com.team254.lib.util.*;
import com.team254.lib.wpilib.TimedRobot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

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
    private final Limelight mLimelight = new Limelight(
            LimelightConstantsFactory.getConstantsForThisRobot(),
            Constants.kLowRes1xZoom);
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Serializer mSerializer = Serializer.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Canifier mCanifier = Canifier.getInstance();
    private final WOF mWOF = WOF.getInstance();


    private Compressor mCompressor;

    private final RobotState mRobotState = RobotState.getInstance();

    private DelayedBoolean mShouldNotShoot;
    private ClimbingStateMachine mClimbingStateMachine = new ClimbingStateMachine();
    private LatchedBoolean mHangModeEnablePressed = new LatchedBoolean();
    private LatchedBoolean mWOFModeEnablePressed = new LatchedBoolean();
    private boolean mInHangMode = false;
    private boolean mInWOFMode = false;
    private double mDisabledStartTime = Double.NaN;

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
                mCanifier,
                mWOF
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
            mShouldNotShoot = new DelayedBoolean(Timer.getFPGATimestamp(), 0.5);

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

            mDisabledStartTime = Timer.getFPGATimestamp();
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

            mInHangMode = false;

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

            mInHangMode = false;
            mClimbingStateMachine.reset();
            mHangModeEnablePressed.update(true);
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

            if ((Timer.getFPGATimestamp() - mDisabledStartTime) > 5.0 &&
                    (Timer.getFPGATimestamp() - mDisabledStartTime) < 5.5) {
                System.out.println("Setting coast!");
                mClimbingStateMachine.reset();
                mDrive.setBrakeMode(false);
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
            boolean wants_shoot = !mShouldNotShoot.update(Timer.getFPGATimestamp(), !mControlBoard.getShoot());

            if (mControlBoard.getZeroGyro()) {
                mDrive.setHeading(Rotation2d.fromDegrees(180));
            }

            mDrive.setHighGear(!mControlBoard.getWantsLowGear());
            // mDrive.setVelocity(VelocityCheesyDriveHelper.getInstance().cheesyDrive(mControlBoard.getThrottle(),
                    // mControlBoard.getTurn(), mControlBoard.getQuickTurn(), !mControlBoard.getWantsLowGear()));
            mDrive.setOpenLoop(OpenLoopCheesyDriveHelper.getInstance().cheesyDrive(mControlBoard.getThrottle(),
                    mControlBoard.getTurn(), mControlBoard.getQuickTurn()));

            boolean hangModePressed =
                    mHangModeEnablePressed.update(mControlBoard.getToggleHangMode());

            if (hangModePressed && !mInHangMode) {
                System.out.println("Entering hang mode!!!!");
                mInHangMode = true;
            } else if (hangModePressed && mInHangMode) {
                System.out.println("Exiting hang mode!");
                mInHangMode = false;
                mClimbingStateMachine.reset();
            }

            boolean WOFModePressed = mWOFModeEnablePressed.update(mControlBoard.getToggleWOFMode());
            if (WOFModePressed && !mInWOFMode) {
                System.out.println("Entering WOF mode!!!!");
                mInWOFMode = true;
            } else if (WOFModePressed && mInWOFMode) {
                System.out.println("Exiting WOF mode!");
                mInWOFMode = false;
            }


            if (mInWOFMode) {
                mWOF.setDeploy(true);
                mWOF.setOpenLoop(mControlBoard.getStir() * 0.5);
            } else {
                mWOF.setDeploy(false);
            }


            if (mInHangMode) {
                mClimbingStateMachine.handle(Timer.getFPGATimestamp(), mControlBoard.getClimbJog(), false, mControlBoard.getRetractIntake(),
                        mControlBoard.getHumanPlayerIntake(), mControlBoard.getDeployIntake());
                mSuperstructure.setTurretHintRobotRelative(Timer.getFPGATimestamp(), -90);
            } else {
                //check getExhaust first so can exh while intake is down by pressing both r/l bumpers?
                if (Math.abs(mControlBoard.getStir()) > Constants.kSerializerStirDeadband && !mInWOFMode) {
                    mSerializer.setStirOverriding(true);
                    mSerializer.setOpenLoop(Util.handleDeadband(mControlBoard.getStir(),
                            Constants.kSerializerStirDeadband) * Constants.kSerializerStirScalar);
                } else {
                    mSerializer.setStirOverriding(false);
                }

                if (mControlBoard.getExhaust()) {
                    mIntake.setWantedState(Intake.WantedState.EXHAUST);
                } else if (mControlBoard.getIntake()) {
                    mIntake.setWantedState(Intake.WantedState.INTAKE);
                } else {
                    mIntake.setWantedState(Intake.WantedState.IDLE);
                }

                boolean wants_aim = mControlBoard.getAimCoarse() || mControlBoard.getAimFine();

                if (wants_shoot) {
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
            }

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {}
}
