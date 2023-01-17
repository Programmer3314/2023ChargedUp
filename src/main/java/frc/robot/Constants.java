// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class Constants {
    public static class Conv {
        public static final double metersPerInch = 0.0254;
        public static final double metersPerFeet = 0.3048;
    }

    public static class Falcon {
        public static final double ticksPerRev = 2048.0;
    }

    public static class Neo {
        public static final double ticksPerRev = 42.0;
    }

    public static class MK4i {
        public static final double wheelDiameter = .10033;
        public static final boolean driveIsInverted = true;
        public static final boolean steerIsInverted = true;
        public static final boolean absoluteEncoderIsInverted = false;
        public static final double wheelCircumference = wheelDiameter * Math.PI;
        public static final double wheelOffset = 2.625 * Constants.Conv.metersPerInch;

        public static class L2 {
            public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
            public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);
            public static final double driveMetersPerTick = (wheelCircumference / Falcon.ticksPerRev) * driveReduction;
            public static final double turnRadiansPerTick = (2.0 * Math.PI / Falcon.ticksPerRev) * steerReduction;
            public static final double maxVelocityMetersPerSecond = 4.14528; // With the L2... 4.96824
            public static final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond
                    / Constants.Chassis.wheelDiagnal;
        }
    }

    public static class Robot {
        public static final int canBusTimeoutMs = 30;
        public static final double MAX_VOLTAGE = 12.0;
    }

    public static class Chassis {
        public static final double length = 24.5 * Constants.Conv.metersPerInch;
        public static final double width = 24.5 * Constants.Conv.metersPerInch;
        public static final double trackWidth = width - (2.0 * MK4i.wheelOffset);
        public static final double wheelBase = length - (2.0 * MK4i.wheelOffset);
        public static final double wheelDiagnal = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

        public static final Translation2d[] moduleOffset = new Translation2d[] {
                new Translation2d(Constants.Chassis.wheelBase / 2.0,
                        Constants.Chassis.trackWidth / 2.0),
                new Translation2d(Constants.Chassis.wheelBase / 2.0,
                        -Constants.Chassis.trackWidth / 2.0),
                new Translation2d(-Constants.Chassis.wheelBase / 2.0,
                        -Constants.Chassis.trackWidth / 2.0),
                new Translation2d(-Constants.Chassis.wheelBase / 2.0,
                        Constants.Chassis.trackWidth / 2.0)
        };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                moduleOffset);
    }

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(28);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-87);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(59);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(49);

    public static class Driver {
        public static final int Controller = 4;

        public static class Axis {
            public static final int y = 0;
            public static final int x = 1;
            public static final int r = 4;
        }

        public static class Button {
            public static final int overrideFieldCentricA = 1;
            public static final int resetNavxB = 2;
            public static final int targetTagY = 4;
            public static final int targetPegX = 3;
        }
    }

    public static class ButtonBox1 {
        public static final int button = 1;

        public static class Button {
            public static final int changePipeline = 1;
            public static final int autoDriveToRamp=2;
        }
    }

    public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
            Math.PI / 2.0, Math.PI / 2.0);
}
