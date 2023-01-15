// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Main;

/** Add your docs here. */
public class MMnavigationSubsystem extends SubsystemBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final SwerveDrivePoseEstimator odometer;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable limelight = inst.getTable("limelight");
    private Pose2d mainPose = new Pose2d();
    private Pose2d aprilPose = new Pose2d();
    private int aprilCount = 0;

    public MMnavigationSubsystem(MMSwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        odometer = new SwerveDrivePoseEstimator(Constants.Chassis.kinematics, new Rotation2d(),
                swerveSubsystem.getSwerveModulePositions(), new Pose2d());

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    // TODO: integrate gyro into the mainpose, when we have a good image, adjust the
    // gyro. Have an offset adjustment variable to resetgyro and set gyro to that.
    // TODO: update the mainpose only when we are confident in the value, tons of
    // gittering in the robot, we can keep track of the value but only set it to
    // Mainpose in specific instances.
    @Override
    public void periodic() {
        odometer.update(getRotation2d(), swerveSubsystem.getSwerveModulePositions());
        mainPose = odometer.getEstimatedPosition();
        aprilPose = getLimelightPose();
        syncCamPose();
        mainPose = new Pose2d(mainPose.getTranslation(), navx.getRotation2d());
        SmartDashboard.putNumber("Robot Heading", Math.toDegrees(getHeadingRad()));
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("AprilTag Rotation Degrees", aprilPose.getRotation().getDegrees());
    }

    public void zeroHeading() {
        navx.reset();
    }

    public double getHeadingRad() {
        // return -MathUtil.angleModulus(navx.getAngle() * Math.PI / 180.0);
        return MathUtil.angleModulus(mainPose.getRotation().getRadians());
    }

    public Rotation2d getRotation2d() {
        // return Rotation2d.fromRadians(getHeadingRad());
        return mainPose.getRotation();
    }

    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return mainPose;
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerveSubsystem.getSwerveModulePositions(), pose);
    }

    public Pose2d getLimelightPose() {

        boolean hasTarget = limelight.getEntry("tv").getNumber(0).doubleValue() > 0.5;

        if (hasTarget) {
            double[] def = new double[] { 0, 0, 0, 0, 0, 0 };
            double[] bp = limelight.getEntry("botpose").getDoubleArray(def);
            if (bp.length == 6) {
                // SmartDashboard.putNumber("BP[5]", bp[5]);
                Pose2d tempPose = new Pose2d(bp[0], bp[1], Rotation2d.fromDegrees(bp[5]));
                aprilCount++;
                return new Pose2d((aprilPose.getX() + tempPose.getX()) / 2.0,
                        (aprilPose.getY() + tempPose.getY()) / 2.0,
                        new Rotation2d(tempPose.getRotation().getRadians()));
            }
        }
        aprilCount = 0;
        // SmartDashboard.putNumber("BP[5]", bp[5]);
        return mainPose;
    }

    public void syncCamPose() {
        if (aprilCount >= 10) {
            mainPose = new Pose2d(aprilPose.getTranslation(), aprilPose.getRotation());
            resetOdometry(new Pose2d(mainPose.getTranslation(), aprilPose.getRotation()));

        }

    }
}
