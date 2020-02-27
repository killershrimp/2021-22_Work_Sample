package com.team254.frc2020.limelight.constants;

import com.team254.frc2020.limelight.undistort.UndistortConstants;
import com.team254.frc2020.limelight.undistort.precomputedmaps.UndistortMap_Limelight_1_320x240;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class LimelightConstantsFactory {
    public static LimelightConstants getConstantsForId(int id) {
        switch (id) {
            default: // Intentional fall through
            case 1:
                // Undistort constants calculated 2/22 on 960x720 for LL 2+ #1
                return new LimelightConstants(
                        1, // label id
                        LimelightConstants.Type.Shooter,
                        "Turret Limelight #1", // name
                        "limelight", // table name
                        26.48, // height
                        new Pose2d(-5.7, 0, Rotation2d.fromDegrees(1.5)), // turret to lens
                        Rotation2d.fromDegrees(27.00), // horizontalPlaneToLens,
                        new UndistortMap_Limelight_1_320x240(), //undistort map
                        new UndistortConstants( // undistort constants
                                new double[]{2.03204609e-01, -6.25404962e-01, -3.39277869e-03, -3.51126715e-04, 5.81122457e-01}, // camera distortion
                                new double[][]{ // camera matrix
                                        {0.78474188, 0.0, 0.51036895},
                                        {0.0, 1.04536274, 0.45914132},
                                        {0.0, 0.0, 1.0}
                                })
                );

            case 2:
                // Not calibrated TODO: fill in correct calibration values
                return new LimelightConstants(
                        1, // label id
                        LimelightConstants.Type.Shooter,
                        "Turret Limelight #2", // name
                        "limelight", // table name
                        26.48, // height
                        new Pose2d(-5.7, 0, Rotation2d.fromDegrees(1.5)), // turret to lens
                        Rotation2d.fromDegrees(27.00), // horizontalPlaneToLens,
                        null, // TODO: build static map
                        new UndistortConstants( // undistort constants
                                new double[]{2.03204609e-01, -6.25404962e-01, -3.39277869e-03, -3.51126715e-04, 5.81122457e-01}, // camera distortion
                                new double[][]{ // camera matrix
                                        {0.78474188, 0.0, 0.51036895},
                                        {0.0, 1.04536274, 0.45914132},
                                        {0.0, 0.0, 1.0}
                                })
                );
        }
    }
}
