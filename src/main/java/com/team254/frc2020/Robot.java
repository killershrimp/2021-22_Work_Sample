package com.team254.frc2020;

import java.util.Optional;

import com.team254.frc2020.auto.AutoModeExecutor;
import com.team254.frc2020.auto.modes.AutoModeBase;
import com.team254.frc2020.controlboard.ControlBoard;
import com.team254.frc2020.controlboard.IControlBoard;
import com.team254.frc2020.loops.Looper;
import com.team254.frc2020.paths.TrajectoryGenerator;
import com.team254.frc2020.subsystems.Drive;
import com.team254.frc2020.subsystems.RobotStateEstimator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.CrashTracker;
import com.team254.lib.util.VelocityCheesyDriveHelper;
import com.team254.lib.wpilib.TimedRobot;

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

    private final RobotState mRobotState = RobotState.getInstance();

    Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();

            mSubsystemManager.setSubsystems(RobotStateEstimator.getInstance(), mDrive);

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);
            
            mTrajectoryGenerator.generateTrajectories();

            // Robot starts forwards. (TODO use robotstate turret reset method once readded)
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.setHeading(Rotation2d.identity());

            mAutoModeSelector.updateModeCreator();
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

            // Robot starts forwards. (TODO use robotstate turret reset method once readded)
            mRobotState.reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mDrive.setHeading(Rotation2d.identity());

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
            manualControl(/*sandstorm=*/false);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public void manualControl(boolean sandstorm) {
        mDrive.setHighGear(!mControlBoard.getWantsLowGear());
        mDrive.setVelocity(VelocityCheesyDriveHelper.getInstance().cheesyDrive(-mControlBoard.getThrottle(),
                -mControlBoard.getTurn(), mControlBoard.getQuickTurn(), !mControlBoard.getWantsLowGear()));
    }

    @Override
    public void testPeriodic() {}
}
