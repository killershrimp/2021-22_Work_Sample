package com.team254.frc2020.paths;

import com.team254.frc2020.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.CentripetalAccelerationConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryGenerator {
    // TODO tune
    private static final double kMaxVel = 150.0;
    private static final double kMaxAccel = 100.0;
    private static final double kMaxVoltage = 9.0;

    private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public static TrajectoryGenerator getInstance() {
        return mInstance;
    }

    private TrajectoryGenerator() {
        mMotionPlanner = new DriveMotionPlanner();
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<TimingConstraint<Pose2dWithCurvature>> constraints,
            double start_vel,  // inches/s
            double end_vel,  // inches/s
            double max_vel,  // inches/s
            double max_accel,  // inches/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    public class TrajectorySet {
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectory;
        public final Trajectory<TimedState<Pose2dWithCurvature>> testTrajectoryBack;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trajBlueWOF;
        public final Trajectory<TimedState<Pose2dWithCurvature>> trajBlueWOFBack;


        public final Trajectory<TimedState<Pose2dWithCurvature>> startingToPickup;
        public final Trajectory<TimedState<Pose2dWithCurvature>> pickupToTurningPoint;
        public final Trajectory<TimedState<Pose2dWithCurvature>> turningPointToWOF;
        public final Trajectory<TimedState<Pose2dWithCurvature>> WOFtoshootingPoint;



//        public final Trajectory<TimedState<Pose2dWithCurvature>> turnInPlace;

        private TrajectorySet() {
            testTrajectory = getTestTrajectory();
            testTrajectoryBack = getTestTrajectoryBack();
            trajBlueWOF = trajBlueWOF();
            trajBlueWOFBack = trajBlueWOFBack();


            startingToPickup = traj8();
            pickupToTurningPoint = traj5();
            turningPointToWOF = traj6();
            WOFtoshootingPoint = traj7();

//            turnInPlace = getTurnInPlace();
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-120, 120, Rotation2d.fromDegrees(90)));
            return generateTrajectory(false, waypoints, Arrays.asList(),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }


        private Trajectory<TimedState<Pose2dWithCurvature>> getTestTrajectoryBack() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-120, 120, Rotation2d.fromDegrees(90)));
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            return generateTrajectory(true, waypoints, Arrays.asList(),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> trajBlueWOF() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-234, 0, Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> trajBlueWOFBack() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-234, 0, Rotation2d.fromDegrees(180)));
//            waypoints.add(new Pose2d(-168, 0, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-96, 68, Rotation2d.fromDegrees(202.5)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }


// go
        private Trajectory<TimedState<Pose2dWithCurvature>> traj3() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-240, 0, Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        private Trajectory<TimedState<Pose2dWithCurvature>> traj4() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-240, 0, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-96, 68, Rotation2d.fromDegrees(202.5)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        // starting to pickup
        private Trajectory<TimedState<Pose2dWithCurvature>> traj8() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-87, 83, Rotation2d.fromDegrees(113.59)));

            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        // pickup to turning piint
        private Trajectory<TimedState<Pose2dWithCurvature>> traj5() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-87, 83, Rotation2d.fromDegrees(113.59)));
            waypoints.add(new Pose2d(-41, 4, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(60, 4, Rotation2d.fromDegrees(180)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        // turniing poiint to WOF
        private Trajectory<TimedState<Pose2dWithCurvature>> traj6() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(60, 4, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-240, 0, Rotation2d.fromDegrees(180)));
            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

        // WOF point to shooting point
        private Trajectory<TimedState<Pose2dWithCurvature>> traj7() {
            List<Pose2d> waypoints = new ArrayList<>();
            waypoints.add(new Pose2d(-240, 0, Rotation2d.fromDegrees(180)));
            waypoints.add(new Pose2d(-96, 68, Rotation2d.fromDegrees(202.5)));
            return generateTrajectory(true, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
                    kMaxVel, kMaxAccel, kMaxVoltage);
        }

//        private Trajectory<TimedState<Pose2dWithCurvature>> getTurnInPlace() {
//            List<Pose2d> waypoints = new ArrayList<>();
//            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180)));
//            waypoints.add(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(0)));
//            return generateTrajectory(false, waypoints, Arrays.asList(new CentripetalAccelerationConstraint(60)),
//                    kMaxVel, kMaxAccel, kMaxVoltage);
//        }

        // at balls, 10 inches bumpbner  to wall
    }
}