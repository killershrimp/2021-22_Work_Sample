package com.team254.frc2020;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.team254.frc2020.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team254.frc2020.limelight.CameraResolution;
import com.team254.frc2020.limelight.PipelineConfiguration;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.ShootingParameters;

/**
 * A list of constants used by the rest of the robot code. This includes physics
 * constants as well as constants determined through calibration.
 * 
 * Port assignments should match up with the spreadsheet here:
 * https://docs.google.com/spreadsheets/d/1U1r9AyXk8nuGuACa36iQRRekL6HqLXF7dZ437_qiD98/edit?usp=sharing
 */
public class Constants {
    public static final double kLooperDt = 0.01;

    // CAN
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    // Control board
    public static final boolean kUseDriveGamepad = false;
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 0;
    public static final int kMainTurnJoystickPort = 1;
    public static final double kJoystickThreshold = 0.2;
    public static final double kJogTurretScalar = 4.0;

    // Drivebase
    // ids
    public static final int kLeftDriveMaster1Id = 1;
    public static final int kLeftDriveMaster2Id = 2;
    public static final int kLeftDriveMaster3Id = 3;
    public static final int kRightDriveMaster1Id = 4;
    public static final int kRightDriveMaster2Id = 5;
    public static final int kRightDriveMaster3Id = 6;

    public static final int kLeftDriveEncoderA = 0;
    public static final int kLeftDriveEncoderB = 1;
    public static final int kRightDriveEncoderA = 2;
    public static final int kRightDriveEncoderB = 3;

    // Drive ratio.
    public static final double kDriveEncoderPPR = 1000.0;
    public static final double kDriveRotationsPerTickHighGear = 1.0 / 2048.0 * 1.0 / (40.0 / 11.0 * 44.0 / 20.0); // ticks * kDriveRotationsPerTicksHighGear = wheel rotations
    public static final double kDriveRotationsPerTickLowGear = 1.0 / 2048.0 * 1.0 / (40.0 / 11.0 * 50.0 / 14.0); // ticks * kDriveRotationsPerTicksLowGear = wheel rotations
    public static final double kGearRatioScalar = (1.0 / (40.0 / 10.0 * 50.0 / 14.0)) / (1.0 / (40.0 / 11.0 * 50.0 / 14.0)); // TODO this is temporary, remove and just edit PID constants manually
    // Wheel parameters.
    public static final double kDriveWheelTrackWidthInches = 30.0; //tuned 3/2
    public static final double kDriveWheelDiameterInches = 5.9067052758; //tuned 3/2
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 1.0;

    // pidf gains (TODO tune)
    public static final double kDriveHighGearKp = 0.0254 * kGearRatioScalar;
    public static final double kDriveHighGearKi = 0.0;
    public static final double kDriveHighGearKd = 0.0;
    public static final double kDriveHighGearKf = 0.0;
    public static final double kDriveLowGearKp = 0.0;
    public static final double kDriveLowGearKi = 0.0;
    public static final double kDriveLowGearKd = 0.0;
    public static final double kDriveLowGearKf = 0.0;

    // deadband
    public static final double kDriveThrottleDeadband = 0.04;
    public static final double kDriveWheelDeadband = 0.035;


    // robot dynamics (TODO tune)
    public static final double kDriveVIntercept = 0.235 * kGearRatioScalar; // V // todo do i need to do this
    public static final double kDriveLinearKv = 0.0488 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V / rad/s
    public static final double kDriveLinearKa = 0.00597 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V / rad/s^2
    public static final double kDriveAngularKa = 0.00517 / 2.0 * Constants.kDriveWheelDiameterInches * kGearRatioScalar; // V per rad/s^2
    public static final double kRobotLinearInertia = 62.051; // kg // TODO
    public static final double kRobotAngularInertia = kDriveAngularKa / kDriveLinearKa *
            kDriveWheelTrackRadiusWidthMeters * kDriveWheelTrackRadiusWidthMeters * kRobotLinearInertia;  // kg m^2
    public static final double kRobotAngularDrag = 40.0; // N*m / (rad/sec)

    // path following (TODO tune?)
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    // Turret.
    public static final ServoMotorSubsystemConstants kTurretConstants = new ServoMotorSubsystemConstants();
    static {
        kTurretConstants.kName = "Turret";

        kTurretConstants.kMasterConstants.id = 10;
        kTurretConstants.kMasterConstants.invert_motor = false;

        // Unit == Degrees
        kTurretConstants.kHomePosition = 180.0;
        kTurretConstants.kTicksPerUnitDistance = 1.0 / (1.0 / 2048.0 * 8.0 / 24.0 * 14.0 / 240.0 * 360.0);

        kTurretConstants.kKf = 1023.0 * 0.25 / 4650.0; // Tuned 3/4
        kTurretConstants.kPositionKp = 0.21;
        kTurretConstants.kPositionDeadband = (int) (0.5 * kTurretConstants.kTicksPerUnitDistance); // Ticks

        kTurretConstants.kMinUnitsLimit = 20.0;
        kTurretConstants.kMaxUnitsLimit = 340.0;

        // TODO current limits, should recover position on reset?

        kTurretConstants.kRecoverPositionOnReset = true;
    }

    public static final Translation2d kVehicleToTurretTranslation = new Translation2d(-6.9, 0);

    // Hood
    public static final ServoMotorSubsystemConstants kHoodConstants = new ServoMotorSubsystemConstants();
    static {
        kHoodConstants.kName = "Hood";

        kHoodConstants.kMasterConstants.id = 11;
        kHoodConstants.kMasterConstants.invert_motor = true;

        // Unit == Degrees
        kHoodConstants.kHomePosition = 45.0;
        kHoodConstants.kTicksPerUnitDistance = 1.0 / (1.0 / 2048.0 * 8.0 / 40.0 * 16.0 / 38.0 * 14.0 / 366.0 * 360.0);

        kHoodConstants.kPositionKp = 0.254; // TODO (tune better)
        kHoodConstants.kPositionDeadband = (int) (0.5 * kHoodConstants.kTicksPerUnitDistance); // Ticks

        kHoodConstants.kMinUnitsLimit = 45.0;
        kHoodConstants.kMaxUnitsLimit = 70.0;

        // TODO current limits, should recover position on reset?
        kHoodConstants.kRecoverPositionOnReset = true;
    }

    // Shooter
    public static final int kShooterLeftMasterId = 12;
    public static final int kShooterRightMasterId = 13;
    public static final double kShooterKp = 0.0075;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 0.0;
    public static final double kShooterKf = 0.05033127788;
    public static final double kShooterTicksPerRevolution = 2048.0 ; // based on gear reduction between encoder and output shaft, and encoder ppr
    public static final double kShooterAllowableErrorRPM = 250.0; // TODO

    // Serializer
    public static final int kSerializerSpinCycleMasterId = 7;
    public static final int kSerializerLeftRollerMasterId = 8;
    public static final int kSerializerRightRollerMasterId = 9;

    public static final double kSerializerStirDeadband = 0.3;
    public static final double kSerializerStirScalar = 0.5;

    public static final double kFeederRollersKp = 0.075;
    public static final double kFeederRollersKi = 0.0;
    public static final double kFeederRollersKd = 0.2;
    public static final double kFeederRollersKf = 0.04795003;
    public static final double kFeederRollersTicksPerRevolutions = 2048.0 * 1.0/36.0 * 16.0;
    public static final int kFeederAllowableError = 50;
    // Intake
    public static final int kIntakeMasterId = 14;
    public static final double kIntakeStowTime = 0.5; // seconds, time it takes to stow intake
    public static final double kIntakePower = 0.75;
    public static final double kIntakeExhaustPower = -0.5;
    public static final double kLightIntakePower = 0.25; // % output to run after stowing for kLightIntakeTime seconds
    public static final double kLightIntakeTime = 1.0; // seconds, time to run intake for at kLightIntakePower after stowing

    // Canifier
    public static final int kCanifierId = 16;

    // Pigeon IMU
    public static final int kPigeonIMUId = 0;

    // Pneumatics
    public static final int kPCMId = 0;
    public static final int kShifterSolenoidId = 0;
    public static final int kPTOSolenoidId = 1;
    public static final int kIntakeSolenoidId = 2;
    public static final int kSkateParkSolenoidId = 5;
    public static final int kChockSolenoidId = 7;

    // Vision
    public static final boolean kShouldUndistort = true;
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kCameraFrameRate = 90.0; // fps

    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final int kDefaultLimelightId = 1;

    public static final double kVisionTargetHeight = 98.25;
    public static final Pose2d kVisionTargetToGoalOffset = new Pose2d(-29.25, 0, Rotation2d.identity());

    public static final PipelineConfiguration kLowRes1xZoom = new PipelineConfiguration(CameraResolution.F_320x240, 1.0);
    public static final PipelineConfiguration kLowRes2xZoom = new PipelineConfiguration(CameraResolution.F_320x240, 2.0);

    // Shot tuning
    public static final boolean kIsHoodTuning = false;
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kCoarseHoodMap = new InterpolatingTreeMap<>();
    static {
        // 2 point map (Tuned 2/22) TODO tune better and for new feeder
        kCoarseHoodMap.put(new InterpolatingDouble(82.0), new InterpolatingDouble(46.0));
        kCoarseHoodMap.put(new InterpolatingDouble(111.), new InterpolatingDouble(51.));
        kCoarseHoodMap.put(new InterpolatingDouble(134.), new InterpolatingDouble(55.));
        kCoarseHoodMap.put(new InterpolatingDouble(168.), new InterpolatingDouble(58.));
        kCoarseHoodMap.put(new InterpolatingDouble(194.), new InterpolatingDouble(59.5));
        kCoarseHoodMap.put(new InterpolatingDouble(234.), new InterpolatingDouble(61.0));
        kCoarseHoodMap.put(new InterpolatingDouble(250.0), new InterpolatingDouble(62.0));
        kCoarseHoodMap.put(new InterpolatingDouble(258.0), new InterpolatingDouble(62.0));
        kCoarseHoodMap.put(new InterpolatingDouble(268.0), new InterpolatingDouble(63.5));
        kCoarseHoodMap.put(new InterpolatingDouble(301.0), new InterpolatingDouble(63.5));
        kCoarseHoodMap.put(new InterpolatingDouble(346.0), new InterpolatingDouble(64.5));
    }

    public static final ShootingParameters kCoarseShootingParams = new ShootingParameters(
            kCoarseHoodMap,
            Pose2d.identity(),
            4500, // shooter setpoint (rpm)
            100, // shooter allowable error (rpm)
            1.0, // turret allowable error (°)
            0.5 // hood allowable error (°)
    );

    // 3 point map (TODO tune)
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFineHoodMap = new InterpolatingTreeMap<>();
    static {
        // same as above TODO tune better and for new feeder
        kFineHoodMap.put(new InterpolatingDouble(82.0), new InterpolatingDouble(46.0));
        kFineHoodMap.put(new InterpolatingDouble(111.), new InterpolatingDouble(51.));
        kFineHoodMap.put(new InterpolatingDouble(134.), new InterpolatingDouble(55.));
        kFineHoodMap.put(new InterpolatingDouble(168.), new InterpolatingDouble(58.));
        kFineHoodMap.put(new InterpolatingDouble(194.), new InterpolatingDouble(59.5));
        kFineHoodMap.put(new InterpolatingDouble(234.), new InterpolatingDouble(61.0));
        kFineHoodMap.put(new InterpolatingDouble(250.0), new InterpolatingDouble(62.0));
        kFineHoodMap.put(new InterpolatingDouble(258.0), new InterpolatingDouble(62.0));
        kFineHoodMap.put(new InterpolatingDouble(268.0), new InterpolatingDouble(63.5));
        kFineHoodMap.put(new InterpolatingDouble(301.0), new InterpolatingDouble(63.5));
        kFineHoodMap.put(new InterpolatingDouble(346.0), new InterpolatingDouble(64.5));
    }

    public static final ShootingParameters kFineShootingParams = new ShootingParameters(
            kFineHoodMap,
            Constants.kVisionTargetToGoalOffset,
            4500, // shooter setpoint (rpm)
            100, // shooter allowable error (rpm)
            1.0, // turret allowable error (°)
            0.5 // hood allowable error (°)
    );

    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}