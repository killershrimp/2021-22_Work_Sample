package com.team254.frc2020;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.team254.frc2020.subsystems.limelight.CameraResolution;
import com.team254.frc2020.subsystems.limelight.LimelightConstants;
import com.team254.frc2020.subsystems.limelight.PipelineConfiguration;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

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

    // Drivebase
    // ids
    public static final int kLeftDriveMaster1Id = 1;
    public static final int kLeftDriveMaster2Id = 2;
    public static final int kLeftDriveMaster3Id = 3;
    public static final int kRightDriveMaster1Id = 4;
    public static final int kRightDriveMaster2Id = 5;
    public static final int kRightDriveMaster3Id = 6;

    // gear ratios (TODO set)
    public static final double kDriveEncoderPPR = 2048.0;
    public static final double kDriveRotationsPerTickHighGear = 1.0 / 2048.0 * 1.0 / 8.80; // ticks * kDriveRotationsPerTicksHighGear = wheel rotations
    public static final double kDriveRotationsPerTickLowGear = 1.0 / 2048.0 * 1.0 / (40.0 / 10.0 * 50.0 / 14.0); // ticks * kDriveRotationsPerTicksLowGear = wheel rotations

    // wheels (TODO tune)
    public static final double kDriveWheelTrackWidthInches = 25.884; //todo characterize?
    public static final double kDriveWheelDiameterInches = 6.1439046227; //tuned 2/12
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kDriveWheelTrackRadiusWidthMeters = kDriveWheelTrackWidthInches / 2.0 * 0.0254;
    public static final double kTrackScrubFactor = 1.0;

    // pidf gains (TODO tune)
    public static final double kDriveHighGearKp = 0.0;
    public static final double kDriveHighGearKi = 0.0;
    public static final double kDriveHighGearKd = 0.0;
    public static final double kDriveHighGearKf = 0.0;
    public static final double kDriveLowGearKp = 0.0;
    public static final double kDriveLowGearKi = 0.0;
    public static final double kDriveLowGearKd = 0.0;
    public static final double kDriveLowGearKf = 0.0;


    // robot dynamics (TODO tune)
    public static final double kDriveVIntercept = 0.0; // V
    public static final double kDriveLinearKv = 0.0; // V / rad/s
    public static final double kDriveLinearKa = 0.0; // V / rad/s^2
    public static final double kDriveAngularKa = 0.0; // V per rad/s^2
    public static final double kRobotLinearInertia = 0.0; // kg
    public static final double kRobotAngularInertia = kDriveAngularKa / kDriveLinearKa *
            kDriveWheelTrackRadiusWidthMeters * kDriveWheelTrackRadiusWidthMeters * kRobotLinearInertia;  // kg m^2
    public static final double kRobotAngularDrag = 0.0; // N*m / (rad/sec)

    // path following (TODO tune?)
    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    // Turret (TODO tune and set)
    public static final int kTurretMasterId = 10;
    public static final double kTurretDegreesPerTick = 1.0 / 2048.0 * 8.0 / 24.0 * 14.0 / 240.0 * 360.0;
    public static final int kTurretForwardSoftLimitDegrees = 340; // degrees
    public static final int kTurretReverseSoftLimitDegrees = 20; // degrees
    public static final Translation2d kVehicleToTurretTranslation = new Translation2d(-6.9, 0);
    public static final double kTurretStartingPositionDegrees = 180; // degrees

    // turret gains
    public static final double kTurretMotionMagicKp = 0.0;
    public static final double kTurretMotionMagicKi = 0;
    public static final double kTurretMotionMagicKd = 0.0;
    public static final double kTurretMotionMagicKf = 0.04659318637;
    public static final int kTurretCruiseVelocity = 17565; // ticks / 100 ms
    public static final int kTurretAcceleration = 52694; // 3 * cruise vel (ticks / 100 ms / s)
    public static final int kTurretMotionMagicAllowableClosedloopError = 146; // ticks (roughly half a °)

    public static final double kTurretPositionKp = 0.21;
    public static final double kTurretPositionKi = 0;
    public static final double kTurretPositionKd = 0.0;
    public static final double kTurretPositionKf = 0.0;
    public static final int kTurretPositionAllowableClosedloopError = 146; // ticks (roughly half a °)

    // Hood (TODO tune and set)
    public static final int kHoodMasterId = 11;
    public static final double kHoodReverseSoftLimitDegrees = 45; // degrees
    public static final double kHoodForwardSoftLimitDegrees = 70; // degrees
    public static final double kHoodStartingPositionDegrees = 45; // degrees

    // hood gains
    public static final double kHoodDegreesPerTick = 1.0 / 2048.0 * 8.0 / 40.0 * 16.0 / 38.0 * 14.0 / 366.0 * 360.0;
    public static final double kHoodKp = 0.254;
    public static final double kHoodKi = 0.0;
    public static final double kHoodKd = 0.0;
    public static final double kHoodKf = 0.0;
    public static final int kHoodCruiseVelocity = 14293; // ticks / 100ms
    public static final int kHoodAcceleration = 42880; // 3* cruise velocity ticks / 100 ms / s
    public static final int kHoodAllowableClosedloopError = 883;

    // Shooter (TODO tune)
    public static final int kShooterLeftMasterId = 12;
    public static final int kShooterRightMasterId = 13;
    public static final double kShooterKp = 0.0075;
    public static final double kShooterKi = 0.0;
    public static final double kShooterKd = 0.0;
    public static final double kShooterKf = 0.05033127788;
    public static final double kShooterTicksPerRevolution = 2048.0 ; // based on gear reduction between encoder and output shaft, and encoder ppr
    public static final double kShooterAllowableErrorRPM = 1000; // TODO

    // Serializer
    public static final int kSerializerSpinCycleMasterId = 7;
    public static final int kSerializerLeftRollerMasterId = 8;
    public static final int kSerializerRightRollerMasterId = 9;

    // Pigeon IMU
    public static final int kPigeonIMUId = 0;

    // Pneumatics
    public static final int kPCMId = 0;
    public static final int kShifterSolenoidId = 0;

    // Vision
    public static final double kHorizontalFOV = 63.709400992230975; // degrees
    public static final double kVerticalFOV = 49.71097153877655; // degrees
    public static final boolean kShouldUndistort = true;
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kCameraFrameRate = 90.0; // fps

    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

    public static final LimelightConstants kLimelightConstants = new LimelightConstants();
    static {
        kLimelightConstants.kHeight = 26.4611;
        kLimelightConstants.kName = "Turret Limelight";
        kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(27.00);
        kLimelightConstants.kTurretToLens = new Pose2d(-5.8448, 0, Rotation2d.identity());
        kLimelightConstants.kTableName = "limelight";
    }

    // calculated 2/16 on 960x720
    public static final double[] kCameraDistortion = {2.35265164e-01, -6.86035030e-01, 3.10037972e-04, 8.34493852e-05, 6.41764110e-01};
    public static final double[][] kCameraMatrix = {
            {778.08226793/960, 0, 452.8538724/960},
            {0, 777.04925262/720, 351.05573266/720},
            {0, 0, 1.0}
    };

    public static final double kVisionTargetHeight = 98.25;
    public static final Pose2d kVisionTargetToGoalOffset = new Pose2d(29.25, 0, Rotation2d.fromDegrees(0));

    public static final PipelineConfiguration kLowRes1xZoom = new PipelineConfiguration(CameraResolution.F_320x240, 1.0);
    public static final PipelineConfiguration kLowRes2xZoom = new PipelineConfiguration(CameraResolution.F_320x240, 2.0);

    // Shot tuning (TODO tune)
    public static final double kShooterSetpointRPM = 5000;
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<>();
    static {
        kHoodMap.put(new InterpolatingDouble(106.2), new InterpolatingDouble(52.5));
    }

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