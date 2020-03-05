package com.team254.frc2020.auto.modes;

import com.team254.frc2020.Constants;
import com.team254.frc2020.auto.AutoModeEndedException;
import com.team254.frc2020.auto.actions.*;
import com.team254.frc2020.paths.TrajectoryGenerator;
import com.team254.lib.geometry.Rotation2d;

import java.util.List;

public class BlueWOFAutoMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new AutoAimAction(Rotation2d.fromDegrees(30)));
        runAction(new ShootAction(Constants.kCoarseShootingParams, 2));
        runAction(new DeployIntakeAction(true));
        runAction(new RunIntakeAction());
        runAction(new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().trajBlueWOF));
        runAction(new ParallelAction(List.of(
                new DriveTrajectoryAction(TrajectoryGenerator.getInstance().getTrajectorySet().trajBlueWOFBack),
                new AutoAimAction(Rotation2d.fromDegrees(0)),
                new SeriesAction(
                        new WaitAction(.5),
                        new StopIntakingAction()),
                        new DeployIntakeAction(false)
                    )
        ));
        runAction(new ShootAction(Constants.kFineShootingParams, 2));
    }
}