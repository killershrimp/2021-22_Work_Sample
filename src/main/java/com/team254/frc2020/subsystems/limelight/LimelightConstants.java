package com.team254.frc2020.subsystems.limelight;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;

public class LimelightConstants {
    public String kName = "Limelight";
    public String kTableName = "limelight";
    public double kHeight = 0.0;
    public Pose2d kTurretToLens = Pose2d.identity();
    public Rotation2d kHorizontalPlaneToLens = Rotation2d.identity();
}