// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class MMNavigationSubsystem extends SubsystemBase {
    private final MMSwerveSubsystem swerveSubsystem;
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private final SwerveDrivePoseEstimator odometer;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable backLimelight = inst.getTable(Constants.Limelight.bLimelight);
    private final NetworkTable clawLimelight = inst.getTable(Constants.Limelight.clawLimelight);
    private final NetworkTable rightLimelight = inst.getTable(Constants.Limelight.rlimelight);
    private final NetworkTable leftLimelight = inst.getTable(Constants.Limelight.lLimelight);
    private Pose2d mainPose = new Pose2d();
    private Pose2d aprilPose = new Pose2d();
    private boolean visionInitialized = false;
    private long leftLastVisionHB;
    private long rightLastVisionHB;
    public static AnalogInput ultraSonicSensor;
    private final Field2d m_field = new Field2d();
    private double angleOffset;
    // private final DigitalInput magneticSensor;
    // private final SendableChooser fieldWidget;

    public MMNavigationSubsystem(MMSwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        odometer = new SwerveDrivePoseEstimator(Constants.Chassis.kinematics, new Rotation2d(),
                swerveSubsystem.getSwerveModulePositions(), new Pose2d());

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                // zeroHeading(() -> true);
            } catch (Exception e) {
            }
        }).start();
        SmartDashboard.putData("Field", m_field);
        ultraSonicSensor = new AnalogInput(Constants.RoboRio.Analog.ultraSonicSensor);
        // Shuffleboard.getTab("In Match").addString("Robo Pose", () ->
        // getPose().toString())
        // .withWidget(BuiltInWidgets.kField);
        // magneticSensor=new DigitalInput(Constants.RoboRio.Dio.magneticSensor);
    }

    @Override
    public void periodic() {
        m_field.setRobotPose(convertPoseToField(getPose()));
        SmartDashboard.putString("onField Pos", convertPoseToField(getPose()).toString());
        odometer.update(getRotation2d(), swerveSubsystem.getSwerveModulePositions());
        mainPose = odometer.getEstimatedPosition();
        aprilPose = getLimelightPose();
        mainPose = new Pose2d(mainPose.getTranslation(), navx.getRotation2d());
        SmartDashboard.putNumber("Robot Heading", Math.toDegrees(getHeadingRad()));
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        // SmartDashboard.putNumber("AprilTag Rotation Degrees",
        // aprilPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Navx Roll", navx.getRoll());

        SmartDashboard.putNumber("TX claw", getClawTargetX());
        SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
        SmartDashboard.putNumber("NavX Pitch", getPitch());
        SmartDashboard.putNumber("Horizontal Distance to Target(Inches)", horizontalDistanceToTarget());
        SmartDashboard.putNumber("Distance to Target(Inches)", distancetoTargetClawCam());

        // SmartDashboard.putNumber("UltraSonicSensor", ultraSonicSensor.getVoltage());
        // SmartDashboard.putBoolean("Magnetic Sensor", magneticSensor.get());
    }

    public void zeroHeading(Supplier<Boolean> isFacingRed) {
        navx.reset();
        setOffset(isFacingRed.get() ? 0 : Math.PI);
    }

    public void setOffset(double offset) {
        angleOffset = offset;
    }

    public double getHeadingRad() {
        return MathUtil.angleModulus(mainPose.getRotation().getRadians());
    }

    public Rotation2d getRotation2d() {
        return mainPose.getRotation();
    }

    public double getUltraSonicVoltage() {
        return ultraSonicSensor.getVoltage();
    }

    public boolean approachingLoadingDock() {
        return getUltraSonicVoltage() < 1;
    }

    public Pose2d getPose() {
        return mainPose;
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(navx.getYaw() + Math.toDegrees(angleOffset));
    }

    public double getRoll() {
        return navx.getRoll();
    }

    public double getPitch() {
        return -navx.getPitch();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerveSubsystem.getSwerveModulePositions(), pose);
    }

    public double getBackTargetX() {
        return backLimelight.getEntry("tx").getNumber(0).doubleValue();
    }

    public double getClawTargetX() {
        return clawLimelight.getEntry("tx").getNumber(0).doubleValue();
    }

    public double distancetoTargetClawCam() {
        double targetOffsetAngleVertical = clawLimelight.getEntry("ty").getDouble(0.0);
        double limelightMountAngleDegrees = 21;
        double goalHeightInches = 18.217;
        double limelightLensHeightInches = 9;
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngleVertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

    public boolean hasTargetClaw() {
        return clawLimelight.getEntry("tv").getNumber(0).doubleValue() > 0.5;
    }

    public boolean hasTargetBack() {
        return backLimelight.getEntry("tv").getNumber(0).doubleValue() > 0.5;
    }

    public double horizontalDistanceToTarget() {
        return distancetoTargetClawCam() * Math.sin(getClawTargetX());
    }

    public Pose2d getLimelightPose() {

        boolean hasTargetLeft = leftLimelight.getEntry("tv").getNumber(0).doubleValue() > 0.5;
        boolean hasTargetRight = rightLimelight.getEntry("tv").getNumber(0).doubleValue() > 0.5;

        if (hasTargetLeft) {
            long currentVisionHB = leftLimelight.getEntry("hb").getInteger(leftLastVisionHB);
            if (currentVisionHB != leftLastVisionHB) {
                leftLastVisionHB = currentVisionHB;
                double[] def = new double[] { 0, 0, 0, 0, 0, 0 };
                double[] bp = leftLimelight.getEntry("botpose").getDoubleArray(def);
                if (bp.length == 6) {

                    Pose2d tempPose = new Pose2d(bp[0], bp[1], Rotation2d.fromDegrees(bp[5]));

                    if (!visionInitialized) {
                        visionInitialized = true;
                        odometer.resetPosition(Rotation2d.fromDegrees(navx.getYaw()),
                                swerveSubsystem.getSwerveModulePositions(), tempPose);
                    }
                    // if (tempPose.getTranslation().getDistance(aprilPose.getTranslation()) < 1) {
                    // SmartDashboard.putNumber("BP[5]", bp[5]);
                    double latency = leftLimelight.getEntry("tl").getDouble(0);
                    latency = (latency + 11) / 1000.0;
                    odometer.addVisionMeasurement(tempPose, Timer.getFPGATimestamp() - latency);
                    // return new Pose2d((aprilPose.getX() + tempPose.getX()) / 2.0,
                    // (aprilPose.getY() + tempPose.getY()) / 2.0,
                    // new Rotation2d(tempPose.getRotation().getRadians()));
                    // // }
                }
            }
        }
        if (hasTargetRight) {
            long currentVisionHB = rightLimelight.getEntry("hb").getInteger(rightLastVisionHB);
            if (currentVisionHB != rightLastVisionHB) {
                rightLastVisionHB = currentVisionHB;
                double[] def = new double[] { 0, 0, 0, 0, 0, 0 };
                double[] bp = rightLimelight.getEntry("botpose").getDoubleArray(def);
                if (bp.length == 6) {

                    Pose2d tempPose = new Pose2d(bp[0], bp[1], Rotation2d.fromDegrees(bp[5]));

                    if (!visionInitialized) {
                        visionInitialized = true;
                        odometer.resetPosition(Rotation2d.fromDegrees(navx.getYaw()),
                                swerveSubsystem.getSwerveModulePositions(), tempPose);
                    }
                    // if (tempPose.getTranslation().getDistance(aprilPose.getTranslation()) < 1) {
                    // SmartDashboard.putNumber("BP[5]", bp[5]);
                    double latency = rightLimelight.getEntry("tl").getDouble(0);
                    latency = (latency + 11) / 1000.0;
                    odometer.addVisionMeasurement(tempPose, Timer.getFPGATimestamp() - latency);
                    // return new Pose2d((aprilPose.getX() + tempPose.getX()) / 2.0,
                    // (aprilPose.getY() + tempPose.getY()) / 2.0,
                    // new Rotation2d(tempPose.getRotation().getRadians()));
                    // }
                }
            }
        }
        return mainPose;
    }

    public void setBackPipeline(int pipelineNumber) {
        backLimelight.getEntry("pipeline").setNumber(pipelineNumber);
    }

    public void setClawPipeline(int pipelineNumber) {
        clawLimelight.getEntry("pipeline").setNumber(pipelineNumber);
    }

    public Pose2d convertPoseToField(Pose2d position) {
        double x = position.getX() + 7.90;
        double y = position.getY() + 3;
        return new Pose2d(x, y, position.getRotation());
    }

}
