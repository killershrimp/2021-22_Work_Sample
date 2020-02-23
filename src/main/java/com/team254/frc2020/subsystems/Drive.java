package com.team254.frc2020.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.team254.frc2020.Constants;
import com.team254.frc2020.RobotState;
import com.team254.frc2020.loops.ILooper;
import com.team254.frc2020.loops.Loop;
import com.team254.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.drivers.TalonFXFactory;
import com.team254.lib.drivers.TalonUtil;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.DriveOutput;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends Subsystem {
    private static Drive mInstance;

    // hardware
    private final TalonFX mLeftMaster1, mRightMaster1, mLeftMaster2, mRightMaster2, mLeftMaster3, mRightMaster3;
    private final Solenoid mShifter;
    private final Encoder mLeftEncoder, mRightEncoder;

    // control states
    private DriveControlState mDriveControlState;
    private PigeonIMU mPigeon;

    // hardware states
    private boolean mIsHighGear;
    private boolean mIsBrakeMode;
    private Rotation2d mGyroOffset = Rotation2d.identity();

    private DriveMotionPlanner mMotionPlanner;
    private boolean mOverrideTrajectory = false;

    private int kHighGearPIDSlot = 0;
    private int kLowGearPIDSlot = 1;

    public synchronized static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    private void configureTalon(TalonFX talon, boolean left, boolean main_encoder_talon) {
        // general
        talon.setInverted(!left);
        talon.configForwardSoftLimitEnable(false);
        talon.configReverseSoftLimitEnable(false);

        // pid
        TalonUtil.checkError(talon.config_kP(kHighGearPIDSlot, Constants.kDriveHighGearKp, Constants.kLongCANTimeoutMs), "Could not set high gear kp");
        TalonUtil.checkError(talon.config_kI(kHighGearPIDSlot, Constants.kDriveHighGearKi, Constants.kLongCANTimeoutMs), "Could not set high gear ki");
        TalonUtil.checkError(talon.config_kD(kHighGearPIDSlot, Constants.kDriveHighGearKd, Constants.kLongCANTimeoutMs), "Could not set high gear kd");
        TalonUtil.checkError(talon.config_kF(kHighGearPIDSlot, Constants.kDriveHighGearKf, Constants.kLongCANTimeoutMs), "Could not set high gear kf");

        TalonUtil.checkError(talon.config_kP(kLowGearPIDSlot, Constants.kDriveLowGearKp, Constants.kLongCANTimeoutMs), "Could not set low gear kp");
        TalonUtil.checkError(talon.config_kI(kLowGearPIDSlot, Constants.kDriveLowGearKi, Constants.kLongCANTimeoutMs), "Could not set low gear ki");
        TalonUtil.checkError(talon.config_kD(kLowGearPIDSlot, Constants.kDriveLowGearKd, Constants.kLongCANTimeoutMs), "Could not set low gear kd");
        TalonUtil.checkError(talon.config_kF(kLowGearPIDSlot, Constants.kDriveLowGearKf, Constants.kLongCANTimeoutMs), "Could not set low gear kf");
        
        // voltage comp
        TalonUtil.checkError(talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs), "could not config drive voltage comp saturation");
        talon.enableVoltageCompensation(true);

        if (main_encoder_talon) {
            // status frames (maybe set for characterization?)
            // TalonUtil.checkError(talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, Constants.kLongCANTimeoutMs), "could not set drive feedback frame");
            // TalonUtil.checkError(talon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 10, Constants.kLongCANTimeoutMs), "could not set drive voltage frame");

            // velocity measurement
            TalonUtil.checkError(talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs), "could not config drive velocity measurement period");
            TalonUtil.checkError(talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs), "could not config drive velocity measurement window");
        }
    }

    private Drive() {
        mPeriodicIO = new PeriodicIO();

        // start all Talons in open loop mode
        mLeftMaster1 = TalonFXFactory.createDefaultTalon(Constants.kLeftDriveMaster1Id);
        configureTalon(mLeftMaster1, true, true);

        mLeftMaster2 = TalonFXFactory.createDefaultTalon(Constants.kLeftDriveMaster2Id);
        configureTalon(mLeftMaster2, true, false);

        mLeftMaster3 = TalonFXFactory.createDefaultTalon(Constants.kLeftDriveMaster3Id);
        configureTalon(mLeftMaster3, true, false);

        mRightMaster1 = TalonFXFactory.createDefaultTalon(Constants.kRightDriveMaster1Id);
        configureTalon(mRightMaster1, false, true);

        mRightMaster2 = TalonFXFactory.createDefaultTalon(Constants.kRightDriveMaster2Id);
        configureTalon(mRightMaster2, false, false);

        mRightMaster3 = TalonFXFactory.createDefaultTalon(Constants.kRightDriveMaster3Id);
        configureTalon(mRightMaster3, false, false);

        mShifter = new Solenoid(Constants.kPCMId, Constants.kShifterSolenoidId);

        mPigeon = new PigeonIMU(Constants.kPigeonIMUId);
        mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 10, 10);

        // force a solenoid message
        mIsHighGear = false;
        setHighGear(true);

        setOpenLoop(DriveSignal.NEUTRAL);

        // force a CAN message across
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();

        mLeftEncoder = new Encoder(Constants.kLeftDriveEncoderA, Constants.kLeftDriveEncoderB, false);
        mRightEncoder = new Encoder(Constants.kRightDriveEncoderA, Constants.kRightDriveEncoderB, true);

        mLeftEncoder.setReverseDirection(true);
        mRightEncoder.setReverseDirection(false);


        resetEncoders();
    }

    private PeriodicIO mPeriodicIO;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double left_voltage;
        public double right_voltage;
        public int left_position_ticks; // using us digital encoder
        public int right_position_ticks; // us digital encoder
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms; // using talonfx
        public int right_velocity_ticks_per_100ms; // talonfx
        public double left_velocity_in_per_sec;
        public double right_velocity_in_per_sec;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.left_voltage = mLeftMaster1.getMotorOutputVoltage();
        mPeriodicIO.right_voltage = mRightMaster1.getMotorOutputVoltage();

        mPeriodicIO.left_position_ticks = mLeftEncoder.get();
        mPeriodicIO.right_position_ticks = mRightEncoder.get();

        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

        mPeriodicIO.left_distance = rotationsToInches(mPeriodicIO.left_position_ticks * getRotationsPerTickDistance());
        mPeriodicIO.right_distance = rotationsToInches(mPeriodicIO.right_position_ticks * getRotationsPerTickDistance());

        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster1.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster1.getSelectedSensorVelocity(0);

        mPeriodicIO.left_velocity_in_per_sec = getLeftLinearVelocity();
        mPeriodicIO.right_velocity_in_per_sec = getRightLinearVelocity();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            mLeftMaster1.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mLeftMaster2.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
            mLeftMaster3.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);

            mRightMaster1.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
            mRightMaster2.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
            mRightMaster3.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
        } else if (mDriveControlState == DriveControlState.VELOCITY || mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            double kd = isHighGear() ? Constants.kDriveHighGearKd : Constants.kDriveLowGearKd;

            mLeftMaster1.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + kd * mPeriodicIO.left_accel / 1023.0);
            mLeftMaster2.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + kd * mPeriodicIO.left_accel / 1023.0);
            mLeftMaster3.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + kd * mPeriodicIO.left_accel / 1023.0);

            mRightMaster1.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + kd * mPeriodicIO.right_accel / 1023.0);
            mRightMaster2.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + kd * mPeriodicIO.right_accel / 1023.0);
            mRightMaster3.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + kd * mPeriodicIO.right_accel / 1023.0);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    stop();
                    setBrakeMode(true);
                    startLogging();
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mDriveControlState) {
                        case OPEN_LOOP:
                            break;
                        case VELOCITY:
                            break;
                        case PATH_FOLLOWING:
                            updatePathFollower();
                            break;
                        default:
                            System.out.println("unexpected drive control state: " + mDriveControlState);
                            break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
                stopLogging();
            }
        });
    }

    public static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesToRadians(double inches) {
        return inches * 2.0 / Constants.kDriveWheelDiameterInches;
    }

    public static double radiansToInches(double radians) {
        return radians / 2.0 * Constants.kDriveWheelDiameterInches;
    }

    public static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    public static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    public static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    /**
     * @param rad_s of the output
     * @return ticks per 100 ms of the talonfx encoder
     */
    private double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) / getRotationsPerTickVelocity() / 10.0;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            setBrakeMode(true);
            System.out.println("switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }

        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    /**
     * Configure talons for velocity control during teleop
     */
    public synchronized void setVelocity(DriveOutput output) {
        if (mDriveControlState != DriveControlState.VELOCITY) {
            setBrakeMode(true);
            System.out.println("switching to velocity");
            mDriveControlState = DriveControlState.VELOCITY;
            configureTalonPIDSlot();
        }

        mPeriodicIO.left_demand = radiansPerSecondToTicksPer100ms(output.left_velocity);
        mPeriodicIO.right_demand = radiansPerSecondToTicksPer100ms(output.right_velocity);
        mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
        mPeriodicIO.left_feedforward = Util.epsilonEquals(mPeriodicIO.left_demand, 0.0) ? 0.0 :
                output.left_feedforward_voltage / 12.0;
        mPeriodicIO.right_feedforward = Util.epsilonEquals(mPeriodicIO.right_demand, 0.0) ? 0.0 :
                output.right_feedforward_voltage / 12.0;
    }

    /**
     * Configure talons for following via the ramsete controller
     */
    public synchronized void setRamseteVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            setBrakeMode(true);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            configureTalonPIDSlot();
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public void overrideTrajectory(boolean value) {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveOutput output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setRamseteVelocity(new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity), radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setRamseteVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public boolean isHighGear() {
        return mIsHighGear;
    }

    public synchronized void configureTalonPIDSlot() {
        int desired_slot_idx = isHighGear() ? kHighGearPIDSlot : kLowGearPIDSlot;

        mLeftMaster1.selectProfileSlot(desired_slot_idx, 0);
        mLeftMaster2.selectProfileSlot(desired_slot_idx, 0);
        mLeftMaster3.selectProfileSlot(desired_slot_idx, 0);

        mRightMaster1.selectProfileSlot(desired_slot_idx, 0);
        mRightMaster2.selectProfileSlot(desired_slot_idx, 0);
        mRightMaster3.selectProfileSlot(desired_slot_idx, 0);
    }

    public synchronized void setHighGear(boolean wantsHighGear) {
        if (wantsHighGear != mIsHighGear) {
            mIsHighGear = wantsHighGear;
            // Plumbed default high.
            mShifter.set(!wantsHighGear);
            configureTalonPIDSlot();
        }
    }

    public boolean isBrakeMode() {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean shouldEnable) {
        if (mIsBrakeMode != shouldEnable) {
            mIsBrakeMode = shouldEnable;
            NeutralMode mode = shouldEnable ? NeutralMode.Brake : NeutralMode.Coast;

            mLeftMaster1.setNeutralMode(mode);
            mLeftMaster2.setNeutralMode(mode);
            mLeftMaster3.setNeutralMode(mode);

            mRightMaster1.setNeutralMode(mode);
            mRightMaster2.setNeutralMode(mode);
            mRightMaster3.setNeutralMode(mode);
        }
    }

    public synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("set heading: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    public synchronized void resetEncoders() {

        mLeftEncoder.reset();
        mRightEncoder.reset();

        mLeftMaster1.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        mLeftMaster2.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        mLeftMaster3.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);

        mRightMaster1.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        mRightMaster2.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);
        mRightMaster3.setSelectedSensorPosition(0, 0, Constants.kCANTimeoutMs);

        mPeriodicIO = new PeriodicIO();
    }

    public double getLeftEncoderDistance() {
        return mPeriodicIO.left_distance;
    }

    public double getRightEncoderDistance() {
        return mPeriodicIO.right_distance;
    }

    public double getRightVelocityNativeUnits() {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity() {
        return rotationsToInches(getRightVelocityNativeUnits() * 10 * getRotationsPerTickVelocity());
    }

    public double getLeftVelocityNativeUnits() {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity() {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10 * getRotationsPerTickVelocity());
    }

    public double getLinearVelocity() {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAverageDriveVelocityMagnitude() {
        return (Math.abs(getLeftLinearVelocity()) + Math.abs(getRightLinearVelocity())) / 2.0;
    }

    public double getAngularVelocity() {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public double getLeftOutputVoltage() {
        return mPeriodicIO.left_voltage;
    }

    public double getRightOutputVoltage() {
        return mPeriodicIO.right_voltage;
    }

    public double getAverageOutputVoltageMagnitude() {
        return (Math.abs(getLeftOutputVoltage()) + Math.abs(getRightOutputVoltage())) / 2.0;
    }

    /**
     * @return conversion factor where ticks * getEncoderTicksPerRotation() = wheel rotations
     */
    public double getRotationsPerTickVelocity() { // talonfx
        return isHighGear() ? Constants.kDriveRotationsPerTickHighGear : Constants.kDriveRotationsPerTickLowGear;
    }

    public double getRotationsPerTickDistance() { // us digital encoders
        return 1.0 / Constants.kDriveEncoderPPR;
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control,
        VELOCITY, // velocity control
        PATH_FOLLOWING
    }

    public enum ShifterState {
        FORCE_LOW_GEAR, FORCE_HIGH_GEAR
    }

    @Override
    public void zeroSensors() {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public boolean checkSystem() {
        // setBrakeMode(false);
        // setHighGear(true);

        // boolean leftSide = SparkMaxChecker.checkMotors(this,
        //     new ArrayList<MotorChecker.MotorConfig<CANSparkMax>>() {
        //         private static final long serialVersionUID = 3643247888353037677L;

        //         {
        //             add(new MotorChecker.MotorConfig<>("left_master", mLeftMaster));
        //             add(new MotorChecker.MotorConfig<>("left_slave", mLeftSlave));
        //         }
        //     }, new MotorChecker.CheckerConfig() {
        //         {
        //             mCurrentFloor = 3;
        //             mRPMFloor = 90;
        //             mCurrentEpsilon = 2.0;
        //             mRPMEpsilon = 200;
        //             mRPMSupplier = mLeftEncoder::getRate;
        //         }
        //     });
        // boolean rightSide = SparkMaxChecker.checkMotors(this,
        //     new ArrayList<MotorChecker.MotorConfig<CANSparkMax>>() {
        //         private static final long serialVersionUID = -1212959188716158751L;

        //         {
        //             add(new MotorChecker.MotorConfig<>("right_master", mRightMaster));
        //             add(new MotorChecker.MotorConfig<>("right_slave", mRightSlave));
        //         }
        //     }, new MotorChecker.CheckerConfig() {
        //         {
        //             mCurrentFloor = 5;
        //             mRPMFloor = 90;
        //             mCurrentEpsilon = 2.0;
        //             mRPMEpsilon = 20;
        //             mRPMSupplier = mRightEncoder::getRate;
        //         }
        //     });

        // return leftSide && rightSide;

        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
        SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
        SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
        SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
        SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
        SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

        SmartDashboard.putNumber("Left Drive Demand", mPeriodicIO.left_demand);
        SmartDashboard.putNumber("Right Drive Demand", mPeriodicIO.right_demand);
        SmartDashboard.putNumber("Left Drive Feedforward", mPeriodicIO.left_feedforward);
        SmartDashboard.putNumber("Right Drive Feedforward", mPeriodicIO.right_feedforward);

        if (getHeading() != null) {
            SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
        }

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }
    }

    public synchronized double getTimestamp() {
        return mPeriodicIO.timestamp;
    }
}