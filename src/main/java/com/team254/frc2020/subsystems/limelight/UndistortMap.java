package com.team254.frc2020.subsystems.limelight;

import com.team254.frc2020.Constants;
import com.team254.lib.util.Util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.opencv.core.CvType.CV_64FC1;
import static org.opencv.core.CvType.CV_64FC2;

public class UndistortMap {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private int width;
    private int height;
    double[][][] map;

    private AtomicBoolean loaded = new AtomicBoolean(false);

    public UndistortMap(int width, int height, boolean loadAsync) {
        this.width = width;
        this.height = height;
        this.map = new double[width][height][2];
        if (loadAsync) {
            new Thread(this::load).start();
        } else {
            load();
        }
    }

    private void load() {
        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++) {
                // run undistort
                double[] undistorted = undistortFromOpenCV(new double[]{i * 1.0 / width * 1.0, j * 1.0 / height * 1.0});
                // output undistorted x and y
                for (int k = 0; k < 2; k++) {
                    map[i][j][k] = (float) undistorted[k];
                }
            }
        }
        loaded.set(true);
    }

    public double[] getUndistortedPoint(double x, double y) {
        if (loaded.get()) {
            x *= width;
            y *= height;
            int intX = (int) Util.limit(x, 0, width);
            int intY = (int) Util.limit(y, 0, height);
            return map[intX][intY];
        } else {
            // Return an undistorted point until this is loaded
            double[] ret = {x, y};
            return ret;
        }
    }

    /**
     * Undoes radial and tangential distortion using opencv
     */
    private static double[] undistortFromOpenCV(double[] point) {
        Mat coord = new Mat(1, 1, CV_64FC2);
        coord.put(0, 0, point);

        Mat camMtx = new Mat(3, 3, CV_64FC1);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                camMtx.put(i, j, Constants.kCameraMatrix[i][j]);
            }
        }

        Mat distortion = new Mat(1, 5, CV_64FC1);
        for (int i = 0; i < 5; i++) {
            distortion.put(0, i, Constants.kCameraDistortion[i]);
        }

        Mat dst = new Mat(1, 1, CV_64FC2);

        Imgproc.undistortPoints(coord, dst, camMtx, distortion, new Mat(), camMtx);

        return dst.get(0, 0);
    }

    public boolean isLoaded() {
        return loaded.get();
    }
}