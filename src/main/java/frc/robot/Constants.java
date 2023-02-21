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

    public static class targetPositions {
        public static final double fieldXCoordinate = 6.0;

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

        public static class L1 {
            public static final double kfalconDrivetrainKI = 0.000000;
            public static final double kfalconDrivetrainKD = 0.00025;
            public static final double kfalconDrivetrainKP = 0.002500;// 5e-5;
            public static final double kfalconDrivetrainKIz = 0;
            public static final double kfalconDrivetrainKFF = 0.050000;
            public static final double kfalconDrivetrainKMaxOutput = 0.869990;
            public static final double kfalconDrivetrainKMinOutput = -0.869990;
            public static final double driveReduction = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);
            public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);
            public static final double driveMetersPerTick = (wheelCircumference / Falcon.ticksPerRev) * driveReduction;
            public static final double turnRadiansPerTick = (2.0 * Math.PI / Falcon.ticksPerRev) * steerReduction;
            public static final double maxVelocityMetersPerSecond = 13.5 * Constants.Conv.metersPerFeet;// 13.5 max
                                                                                                        // changed
            public static final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond
                    / Constants.Chassis.wheelDiagnal;
        }

        public static class L2A {
            public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
            public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);
            public static final double driveMetersPerTick = (wheelCircumference / Falcon.ticksPerRev) * driveReduction;
            public static final double turnRadiansPerTick = (2.0 * Math.PI / Falcon.ticksPerRev) * steerReduction;
            public static final double maxVelocityMetersPerSecond = 16.3 * Constants.Conv.metersPerFeet;
            public static final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond
                    / Constants.Chassis.wheelDiagnal;
            public static final double kfalconDrivetrainKI = 0.000000;
            public static final double kfalconDrivetrainKD = 0.00025;
            public static final double kfalconDrivetrainKP = 0.002500;// 5e-5;
            public static final double kfalconDrivetrainKIz = 0;
            public static final double kfalconDrivetrainKFF = 0.050000;
            public static final double kfalconDrivetrainKMaxOutput = 0.869990;
            public static final double kfalconDrivetrainKMinOutput = -0.869990;
        }

        public static class L3 {
            public static final double driveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
            public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);
            public static final double driveMetersPerTick = (wheelCircumference / Falcon.ticksPerRev) * driveReduction;
            public static final double turnRadiansPerTick = (2.0 * Math.PI / Falcon.ticksPerRev) * steerReduction;
            public static final double maxVelocityMetersPerSecond = 18.0 * Constants.Conv.metersPerFeet;
            public static final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond
                    / Constants.Chassis.wheelDiagnal;
        }

    }

    public static class Robot {
        public static final int canBusTimeoutMs = 30;
        public static final double MAX_VOLTAGE = 12.0;
    }

    public static class Chassis {
        // public static final double length = 24.5 * Constants.Conv.metersPerInch;
        // public static final double width = 24.5 * Constants.Conv.metersPerInch;
        public static final double width = 27 * Constants.Conv.metersPerInch;
        public static final double length = 33 * Constants.Conv.metersPerInch;
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

        public static class frontLeftModule {
            public static final int driveMotor = 2;
            public static final int steerMotor = 1;
            public static final int absoluteEncoderId = 1;
            public static final double absoluteEncoderOffset = -Math.toRadians(-73);// 28
        }

        public static class frontRightModule {
            public static final int driveMotor = 4;
            public static final int steerMotor = 3;
            public static final int absoluteEncoderId = 2;
            public static final double absoluteEncoderOffset = -Math.toRadians(188);// -87
        }

        public static class backRightModule {
            public static final int driveMotor = 6;
            public static final int steerMotor = 5;
            public static final int absoluteEncoderId = 3;
            public static final double absoluteEncoderOffset = -Math.toRadians(-61);// 49
        }

        public static class backLeftModule {
            public static final int driveMotor = 8;
            public static final int steerMotor = 7;
            public static final int absoluteEncoderId = 4;
            public static final double absoluteEncoderOffset = -Math.toRadians(33);// 59
        }
    }

    public static class Limelight {
        public static final String fLimelight = "limelight-front";
        public static final String bLimelight = "limelight-rear";
    }

    public static final double kfalconDrivetrainKI = 0.000000;
    public static final double kfalconDrivetrainKD = 0.00025;
    public static final double kfalconDrivetrainKP = 0.002500;// 5e-5;
    public static final double kfalconDrivetrainKIz = 0;
    public static final double kfalconDrivetrainKFF = 0.050000;
    public static final double kfalconDrivetrainKMaxOutput = 0.869990;
    public static final double kfalconDrivetrainKMinOutput = -0.869990;

    // public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;
    // public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;
    // public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET =
    // -Math.toRadians(28);

    // public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
    // public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    // public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET =
    // -Math.toRadians(-87);

    // public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8;
    // public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7;
    // public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET =
    // -Math.toRadians(59);

    // public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6;
    // public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5;
    // public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET =
    // -Math.toRadians(49);

    public static class Driver {
        public static final int Controller = 4;

        public static class Axis {
            public static final int y = 0;
            public static final int x = 1;
            public static final int r = 4;
            public static final int lt = 2;
            public static final int rt = 3;

        }

        public static class Button {
            public static final int overrideFieldCentricA = 1;
            public static final int resetNavxB = 2;
            // public static final int testPeg = 3;
            // public static final int autoDelivery=1;

            // public static final int runIntake = 4;
            // public static final int runOutTake = 3;
        }
    }

    public static class ButtonBox1 {
        public static final int button = 1;

        public static class Button {
            // public static final int gridGroup1 = 1;
            // public static final int gridGroup2 = 2;
            // public static final int gridGroup3 = 3;
            // public static final int gridGroupCell1 = 4;
            // public static final int gridGroupCell2 = 5;
            // public static final int gridGroupCell3 = 6;
            // public static final int gridGroupHeightLow = 7;
            // public static final int gridGroupHeightMed = 8;
            // public static final int gridGroupHeightHigh = 9;
            // public static final int testTurnPeg = 10;
            // 11+12 don't work

        }
    }

    public static class ButtonBox2 {
        public static final int button = 0;

        public static class Button {
            public static final int col1 = 1;
            public static final int col2 = 2;
            public static final int col3 = 3;
            public static final int col4 = 4;
            public static final int col5 = 5;
            public static final int col6 = 6;
            public static final int col7 = 7;
            public static final int col8 = 8;
            public static final int col9 = 9;
            public static final int row1 = 10;
            public static final int row2 = 11;
            public static final int row3 = 12;
        }

    }

    public static class Pneumatic {
        public static final int pneumaticHubModule = 1;

        public class LowerIntake {
            public static final int reverseChannel = 0;
            public static final int forwardChannel = 1;
        }

        public class UpperIntake {
            public static final int reverseChannel = 2;
            public static final int forwardChannel = 3;
        }
    }

    public static class RoboRio {
        public static class Analog {
            public static final int ultraSonicSensor = 0;

            public static class IntakeSensors {
                public static final int ultraSonicSensor = 1;
            }
        }

        public static class Dio {
            // public static final int magneticSensor = 9;

            public static class IntakeSensors {
                public static final int beamBreakSensor = 1;
                public static final int breakBeamLeft = 5;
                public static final int breakBeamRight = 6;

                public static final int armHome = 7;
                public static final int armCloseToHome = 8;
                public static final int armFarFromHome = 9;

                public static final int spideySense = 4;
            }
        }
    }

    public static class DeliveryMotor {
        public static class IntakeMotor {
            public static final int intakeMotorCanId = 10;
        }
    }

    public static class Arm {
        public class Gripper {
            public static final int solenoid = 1;
            public static final int forwardChannel = 4;
            public static final int reverseChannel = 5;

        }

        public class Extend {
            public class CanID {
                public static final int Id = 11;
            }

            public class SpeedControl {
                public static final double safety = -0.01*2;
                public static final double speed = -0.05*2;
            }

            public class PositionControl {
                public static final double loading = 0.2;
                public static final double highPeg = 0.4;
                public static final double lowPeg = 0.19;
                public static final double ground = 0.4;
            }

            public class PIDValue {
                public static final double I = 0.000000;
                public static final double D = 0.00025;
                public static final double P = 0.002500;// 5e-5;
                public static final double Iz = 0;
                public static final double FF = 0.050000;
                public static final double maxOutput = 0.869990;
                public static final double minOutput = -0.869990;
            }

        }

        public class Rotation {

            public class CanID {
                public static final int Id = 12;
            }

            public class PositionControl {
                public static final double loading = 0.5;
                public static final double highPeg = 0.8;
                public static final double lowPeg = 0.8;
                public static final double ground = 1.2;
            }

            public class PIDValue {
                public static final double I = 0.000000;
                public static final double D = 0.00025;
                public static final double P = 0.002500*4;// 5e-5;
                public static final double Iz = 0;
                public static final double FF = 0.050000;
                public static final double MaxOutput = 0.869990;
                public static final double MinOutput = -0.869990;
            }

        }

        public static class Encoder {
            public static final int Id = 5;
            public static final double OffsetDegrees = 102;
        }

        public class ConversionFactors {
            public static final double extensionTicksPerMeter = 39.37 * 3 * 2048;
            public static final double rotationTicksPerRadians = 2048 / 48 * 15 / 60 / 2 / Math.PI;
        }
    }

    public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
            Math.PI / 2.0, Math.PI / 2.0);
}
