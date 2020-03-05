package com.team254.lib.util;

import com.team254.frc2020.Constants;
import com.team254.frc2020.Kinematics;
import com.team254.frc2020.subsystems.Drive;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.physics.DifferentialDrive.DriveDynamics;
import com.team254.lib.physics.DifferentialDrive.WheelState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static com.team254.lib.util.Util.handleDeadband;

public class VelocityCheesyDriveHelper {
    private static VelocityCheesyDriveHelper mInstance;

    public static VelocityCheesyDriveHelper getInstance() {
        if (mInstance == null) {
            mInstance = new VelocityCheesyDriveHelper();
        }

        return mInstance;
    }

    private VelocityCheesyDriveHelper() {}

    private static final double kMaxHighGearVelocity = Drive.inchesToRadians(150); // rad/s (TODO tune)
    private static final double kMaxHighGearAcceleration = Drive.inchesToRadians(90); // rad/s^2 (TODO tune)

    private static final double kDt = 0.02; // sec, this is the dt of teleopPeriodic

    private static final double kWheelGain = 0.05; // TODO tune

    private static final DCMotorTransmission kTransmission = new DCMotorTransmission(
        1.0 / Constants.kDriveLinearKv,
        Units.inches_to_meters(Constants.kDriveWheelRadiusInches) * Units.inches_to_meters(Constants
                .kDriveWheelRadiusInches) * Constants.kRobotLinearInertia / (2.0 * Constants.kDriveLinearKa),
        Constants.kDriveVIntercept);

    private static final DifferentialDrive mModel = new DifferentialDrive(
        Constants.kRobotLinearInertia,
        Constants.kRobotAngularInertia,
        Constants.kRobotAngularDrag,
        Units.inches_to_meters(Constants.kDriveWheelDiameterInches / 2.0),
        Units.inches_to_meters(Constants.kDriveWheelTrackWidthInches / 2.0 * Constants.kTrackScrubFactor),
        kTransmission, kTransmission
    );

    private double mLastLeftVel = 0.0, mLastRightVel = 0.0;

    public synchronized DriveOutput cheesyDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear) {
        throttle = Util.handleDeadband(throttle, Constants.kDriveThrottleDeadband);
        wheel = Util.handleDeadband(wheel, Constants.kDriveWheelDeadband);

        // TODO: remap throttle and wheel differently (currently just squared)
        throttle = Math.signum(throttle) * Math.pow(throttle, 2);
        wheel = Math.signum(wheel) * Math.pow(wheel, 2) * kWheelGain;

        DriveSignal velocity = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        velocity = velocity.normalize();

        // TODO add support for low gear
        double scaling_factor = kMaxHighGearVelocity;
        velocity = new DriveSignal(velocity.getLeft() * scaling_factor, velocity.getRight() * scaling_factor);

        DriveOutput ret_val = new DriveOutput();
        ret_val.left_velocity = velocity.getLeft();
        ret_val.right_velocity = velocity.getRight();

        SmartDashboard.putNumber("L demanded velocity", Drive.radiansToInches(ret_val.left_velocity));
        SmartDashboard.putNumber("R demanded velocity", Drive.radiansToInches(ret_val.right_velocity));

        WheelState wheel_velocity = new WheelState(ret_val.left_velocity, ret_val.right_velocity);

        ret_val.left_accel = Util.limit((ret_val.left_velocity - mLastLeftVel) / kDt, kMaxHighGearAcceleration);
        ret_val.right_accel = Util.limit((ret_val.right_velocity - mLastRightVel) / kDt, kMaxHighGearAcceleration);
        
//        WheelState wheel_acceleration = new WheelState(ret_val.left_accel, ret_val.right_accel); TODO
        WheelState wheel_acceleration = new WheelState(0., 0.);

        // TODO add support for low gear
        DriveDynamics dynamics = mModel.solveInverseDynamics(wheel_velocity, wheel_acceleration);
        ret_val.left_feedforward_voltage = dynamics.voltage.left;
        ret_val.right_feedforward_voltage = dynamics.voltage.right;

        mLastLeftVel = ret_val.left_velocity;
        mLastRightVel = ret_val.right_velocity;
        
        return ret_val;
    }

    public synchronized DriveOutput naiveDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear) {
        throttle = Util.handleDeadband(throttle, Constants.kDriveThrottleDeadband);
        wheel = Util.handleDeadband(wheel, Constants.kDriveWheelDeadband);

        // TODO: remap throttle and wheel differently (currently just squared)
        throttle = Math.signum(throttle) * Math.pow(throttle, 2);
        wheel = Math.signum(wheel) * Math.pow(wheel, 2) * kWheelGain;

        DriveSignal velocity = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
        velocity = velocity.normalize();

        // TODO add support for low gear
        double scaling_factor = kMaxHighGearVelocity;
        velocity = new DriveSignal(velocity.getLeft() * scaling_factor, velocity.getRight() * scaling_factor);

        return new DriveOutput(velocity.getLeft(), velocity.getRight(), 0, 0, 0, 0);
    }
}