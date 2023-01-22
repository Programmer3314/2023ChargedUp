// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class MMNavigationSubsystem extends SubsystemBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final SwerveDrivePoseEstimator odometer;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable limelight = inst.getTable(Constants.Limelight.fLimelight);
    private Pose2d mainPose = new Pose2d();
    private Pose2d aprilPose = new Pose2d();
    private boolean visionInitialized = false;
    private long lastVisionHB;

    public MMNavigationSubsystem(MMSwerveSubsystem swerveSubsystem) {
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

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), swerveSubsystem.getSwerveModulePositions());
        mainPose = odometer.getEstimatedPosition();
        aprilPose = getLimelightPose();
        mainPose = new Pose2d(mainPose.getTranslation(), navx.getRotation2d());
        SmartDashboard.putNumber("Robot Heading", Math.toDegrees(getHeadingRad()));
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("AprilTag Rotation Degrees", aprilPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Navx Roll", navx.getRoll());
        SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
        SmartDashboard.putNumber("NavX Pitch", navx.getPitch());
    }

    public void zeroHeading() {
        navx.reset();
    }

    public double getHeadingRad() {
        return MathUtil.angleModulus(mainPose.getRotation().getRadians());
    }

    public Rotation2d getRotation2d() {
        return mainPose.getRotation();
    }

    public Pose2d getPose() {
        return mainPose;
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(navx.getYaw());
    }

    public double getRoll() {
        return navx.getRoll();
    }

    public double getPitch(){
        return -navx.getPitch();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerveSubsystem.getSwerveModulePositions(), pose);
    }

    public Pose2d getLimelightPose() {

        boolean hasTarget = limelight.getEntry("tv").getNumber(0).doubleValue() > 0.5;

        if (hasTarget) {
            long currentVisionHB = limelight.getEntry("hb").getInteger(lastVisionHB);
            if (currentVisionHB != lastVisionHB) {
                lastVisionHB = currentVisionHB;
                double[] def = new double[] { 0, 0, 0, 0, 0, 0 };
                double[] bp = limelight.getEntry("botpose").getDoubleArray(def);
                if (bp.length == 6) {
                    Pose2d tempPose = new Pose2d(bp[0], bp[1], Rotation2d.fromDegrees(bp[5]));
                    if (!visionInitialized) {
                        visionInitialized = true;
                        odometer.resetPosition(Rotation2d.fromDegrees(navx.getYaw()),
                                swerveSubsystem.getSwerveModulePositions(), tempPose);
                    }
                    // SmartDashboard.putNumber("BP[5]", bp[5]);
                    double latency = limelight.getEntry("tl").getDouble(0);
                    latency = (latency + 11) / 1000.0;
                    odometer.addVisionMeasurement(tempPose, Timer.getFPGATimestamp() - latency);
                    return new Pose2d((aprilPose.getX() + tempPose.getX()) / 2.0,
                            (aprilPose.getY() + tempPose.getY()) / 2.0,
                            new Rotation2d(tempPose.getRotation().getRadians()));
                }
            }
        }
        return mainPose;
    }

    public void setPipeline(int pipelineNumber) {
        limelight.getEntry("pipeline").setNumber(pipelineNumber);
        SmartDashboard.putString("Changing Pipeline:", "yes");
    }
}
