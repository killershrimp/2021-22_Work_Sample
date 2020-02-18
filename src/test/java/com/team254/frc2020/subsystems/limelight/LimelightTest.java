package com.team254.frc2020.subsystems.limelight;

import com.team254.frc2020.Constants;
import com.team254.frc2020.RobotState;
import com.team254.frc2020.subsystems.limelight.CameraResolution;
import com.team254.frc2020.subsystems.limelight.Limelight;
import com.team254.frc2020.subsystems.limelight.PipelineConfiguration;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import com.team254.lib.vision.TargetInfo;
import org.junit.Assert;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.JUnit4;
import org.opencv.core.Core;

import java.util.ArrayList;
import java.util.List;

@RunWith(JUnit4.class)
public class LimelightTest {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    /**
     * Tests the Limelight corner extraction method
     * Graph with every test point is here:
     * "https://www.desmos.com/calculator/yyx8rpgulg"
     */
    @Test
    public void testExtractTopCornersFromBoundingBoxes() {
        List<double[]> topCorners1 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{94.0, 98.0, 140.0, 142.0},
                new double[]{65.0, 87.0, 86.0, 65.0});
        Assert.assertArrayEquals(new double[]{94.0, 65.0}, topCorners1.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{142.0, 65.0}, topCorners1.get(1), Util.kEpsilon);

        List<double[]> topCorners2 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{118.0, 127.0, 215.0, 223.0},
                new double[]{151.0, 191.0, 179.0, 136.0});
        Assert.assertArrayEquals(new double[]{118.0, 151.0}, topCorners2.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{223.0, 136.0}, topCorners2.get(1), Util.kEpsilon);

        List<double[]> topCorners3 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{114.0, 108.0, 200.0, 194.0},
                new double[]{161.0, 123.0, 134.0, 175.0});
        Assert.assertArrayEquals(new double[]{108.0, 123.0}, topCorners3.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{200.0, 134.0}, topCorners3.get(1), Util.kEpsilon);

        List<double[]> topCorners4 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{86.0, 94.0, 123.0, 134.0},
                new double[]{44.0, 61.0, 67.0, 46.0});
        Assert.assertArrayEquals(new double[]{86.0, 44.0}, topCorners4.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{134.0, 46.0}, topCorners4.get(1), Util.kEpsilon);

        List<double[]> topCorners5 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{86.0, 94.0, 123.0, 134.0, 100.0, 130.0},
                new double[]{44.0, 61.0, 67.0, 46.0, 50.0, 50.0});
        Assert.assertArrayEquals(new double[]{86.0, 44.0}, topCorners5.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{134.0, 46.0}, topCorners5.get(1), Util.kEpsilon);

        List<double[]> topCorners6 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{86.0, 94.0, 123.0, 134.0, 100.0, 130.0, 93.0, 120.0},
                new double[]{44.0, 61.0, 67.0, 46.0, 50.0, 50.0, 70.0, 90.0});
        Assert.assertArrayEquals(new double[]{86.0, 44.0}, topCorners6.get(0), Util.kEpsilon);
        Assert.assertArrayEquals(new double[]{134.0, 46.0}, topCorners6.get(1), Util.kEpsilon);
    }

    /**
     * Tests distance calculation. All the distance and coordinate data was collected on a development board
     */
    @Test
    public void testGetCameraToVisionTargetPose() throws InterruptedException {
        List<TargetInfo> targets = new ArrayList<>();
        PipelineConfiguration noZoomPipeline = new PipelineConfiguration(CameraResolution.F_320x240, 1.0);
        PipelineConfiguration zoomedPipeline = new PipelineConfiguration(CameraResolution.F_320x240, 2.0);

        Limelight limelight = new Limelight(Constants.kLimelightConstants, Constants.kLowRes1xZoom);
        UndistortMap undistortMap = limelight.getUndistortMap();

        while (!undistortMap.isLoaded()) {
            Thread.sleep(10);
        }

        double cameraHeight = 14.125;
        Rotation2d cameraPitch = Rotation2d.fromDegrees(1.6);

        double acceptedError = 5.0;

        // 15 5"
        List<double[]> topCorners1 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{135, 192, 178, 148},
                new double[]{7, 10, 33, 33}
        );
        targets = Limelight.getRawTargetInfos(topCorners1, noZoomPipeline, targets, undistortMap);
        double[] distances1 = averageTargetInfos(targets, cameraHeight, cameraPitch);
        Assert.assertEquals(15*12 + 5, distances1[0], acceptedError);
        targets.clear();

        // 30' 3"
        List<double[]> topCorners2 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{149,177, 171, 155},
                new double[]{67, 67, 79, 79}
        );
        Limelight.getRawTargetInfos(topCorners2, noZoomPipeline, targets, undistortMap);
        double[] distances2 = averageTargetInfos(targets, cameraHeight, cameraPitch);
        Assert.assertEquals(30*12 + 3, distances2[0], acceptedError);
        targets.clear();

        // 30' 1"
        List<double[]> topCorners3 = Limelight.extractTopCornersFromBoundingBoxes(
                        new double[]{153, 180, 174, 159},
                        new double[]{67, 67, 80, 79}
        );
        Limelight.getRawTargetInfos(topCorners3, noZoomPipeline, targets, undistortMap);
        double[] distances3 = averageTargetInfos(targets, cameraHeight, cameraPitch);
        Assert.assertEquals(30*12 + 1, distances3[0], acceptedError);
        targets.clear();

        // 30' 1", 2x zoom
        List<double[]> topCorners4 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{146, 201, 187, 160},
                new double[]{15, 15, 39, 39}
        );
        targets = Limelight.getRawTargetInfos(topCorners4, zoomedPipeline, targets, undistortMap);
        double[] distances4 = averageTargetInfos(targets, cameraHeight, cameraPitch);
        Assert.assertEquals(30*12 + 1, distances4[0], acceptedError);
        targets.clear();

        // 30' 10" zoomed
        List<double[]> topCorners5 = Limelight.extractTopCornersFromBoundingBoxes(
                        new double[]{163, 218, 206, 176},
                        new double[]{19, 19, 44, 44}
        );
        Limelight.getRawTargetInfos(topCorners5, zoomedPipeline, targets, undistortMap);
        double[] distances5 = averageTargetInfos(targets, cameraHeight, cameraPitch);
        Assert.assertEquals(30*12 + 10, distances5[0], acceptedError);
        targets.clear();

        // 31' 1" no zoom
        List<double[]> topCorners6 = Limelight.extractTopCornersFromBoundingBoxes(
                new double[]{163, 190, 184, 168},
                new double[]{69, 69, 82, 82}
        );
        Limelight.getRawTargetInfos(topCorners6, noZoomPipeline, targets, undistortMap);
        double[] distances6 = averageTargetInfos(targets, cameraHeight, cameraPitch);
        Assert.assertEquals(31*12 + 1, distances6[0], acceptedError);
        targets.clear();
    }

    private double[] averageTargetInfos(List<TargetInfo> targets, double cameraHeight, Rotation2d cameraPitch) {
        double x = 0;
        double y = 0;
        for (TargetInfo target : targets) {
            Translation2d distance = RobotState.getCameraToVisionTargetPose(target, cameraHeight, cameraPitch);
            Assert.assertNotNull(distance);
            x += distance.x();
            y += distance.y();
        }
        return new double[]{x/2, y/2};
    }
}