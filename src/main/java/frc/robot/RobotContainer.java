// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ButtonBox1;
import frc.robot.Constants.ButtonBox2;
import frc.robot.commands.AdjustArmPoseCmd;
import frc.robot.commands.AutoDeliveryCmd;
import frc.robot.commands.OneBallAutoCmd;
import frc.robot.commands.OneConeAutoCmd;
import frc.robot.commands.DeliverCubeCmd;
import frc.robot.commands.DriveToBumperCmd;
import frc.robot.commands.DriveToRampCmd;
import frc.robot.commands.GripGrabCmd;
import frc.robot.commands.GripReleaseCmd;
import frc.robot.commands.LeftDockExtract;
import frc.robot.commands.LockedInCmd;
import frc.robot.commands.ManualDeliverCmd;
import frc.robot.commands.PickUpCubeCmd;
import frc.robot.commands.PositionGroundCmd;
import frc.robot.commands.PositionHighPegCmd;
import frc.robot.commands.PositionHomeCmd;
import frc.robot.commands.PositionLoadingCmd;
import frc.robot.commands.PositionLowPegCmd;
import frc.robot.commands.RightDockExtract;
import frc.robot.commands.RunIntakeUntilBrokenCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TargetPegCmd;
import frc.robot.commands.TargetPegDriveCmd;
import frc.robot.commands.TargetPegLateralCmd;
import frc.robot.commands.TargetTagCmd;
import frc.robot.commands.TargetTagDriveCmd;
import frc.robot.commands.TargetTagLateralCmd;
import frc.robot.commands.TranslateAbsoluteCmd;
import frc.robot.commands.TranslateRelativeCmd;
import frc.robot.commands.TwoBallAutoCmd;
import frc.robot.commands.GripGrabRaw;
import frc.robot.subsystems.MMIntakeSubsystem;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMField;
import frc.robot.utility.MMJoystickAxis;

public class RobotContainer {
        private ShuffleboardTab tab = Shuffleboard.getTab("In Match");

        private final SendableChooser<Integer> getAutoStartPose = new SendableChooser<>();
        private final SendableChooser<Integer> getDesiredHeight = new SendableChooser<>();
        private final SendableChooser<AutonomousSelection> getDesiredAuto = new SendableChooser<>();
        private final SendableChooser<Boolean> getChargingStation = new SendableChooser<>();

        private double manualAngle = 0;
        private double startingExtension = 0;

        private static Alliance alliance;
        private static boolean isRedAlliance = true;

        public final MMSwerveSubsystem swerveSubsystem = new MMSwerveSubsystem();
        public final MMNavigationSubsystem navigationSubsystem = new MMNavigationSubsystem(swerveSubsystem);

        private final Joystick driverJoystick = new Joystick(Constants.Driver.Controller);
        private final Joystick buttonBox1 = new Joystick(Constants.ButtonBox1.button);
        private final Joystick buttonBox2 = new Joystick(0);
        private final MMJoystickAxis driveXAxis = new MMJoystickAxis(Constants.Driver.Controller,
                        Constants.Driver.Axis.x,
                        0.01,
                        -3.3);// -1.3
        private final MMJoystickAxis driveYAxis = new MMJoystickAxis(Constants.Driver.Controller,
                        Constants.Driver.Axis.y,
                        0.01,
                        -3.3);// -1.3
        private final MMJoystickAxis driveRAxis = new MMJoystickAxis(Constants.Driver.Controller,
                        Constants.Driver.Axis.r,
                        0.05,
                        -(Math.PI));/// 2
        private int gridHeight;
        private int gridGroup;
        private int gridGroupCell;
        private int gridCell;
        private GenericEntry gridCellEntry = tab.add("Grid Cell: ", "None").getEntry();
        private Trigger leftTrigger;
        private Trigger rightTrigger;
        public final MMIntakeSubsystem intakeSubsystem;
        private double startingAngle = 0;
        private Pose2d autoPosition;
        private boolean isOverChargingStation;
        private POVButton driverLeftPOV;
        private POVButton driverRightPOV;
        private POVButton driverUpPOV;
        private POVButton driverDownPOV;

        public RobotContainer() {
                getChargingStation.setDefaultOption("True", true);
                getChargingStation.addOption("False", false);
                getDesiredAuto.setDefaultOption("One Ball", AutonomousSelection.OneBallAuto);
                getDesiredAuto.setDefaultOption("One Cone", AutonomousSelection.OneConeAuto);
                getDesiredAuto.addOption("Two Ball", AutonomousSelection.TwoBallAuto);
                getDesiredAuto.addOption("Three Ball", AutonomousSelection.ThreeBallAuto);
                getDesiredHeight.setDefaultOption("None Selected", 0);
                for (int i = 1; i < 4; i++) {
                        getDesiredHeight.addOption("Height: " + i, i);
                }
                getAutoStartPose.setDefaultOption("None Selected", 0);
                for (int i = 1; i < 10; i++) {
                        getAutoStartPose.addOption("Cell: " + i, i);
                }

                Shuffleboard.getTab("In Match").add(getAutoStartPose).withWidget(BuiltInWidgets.kComboBoxChooser);
                Shuffleboard.getTab("In Match").add(getDesiredAuto).withWidget(BuiltInWidgets.kComboBoxChooser);
                Shuffleboard.getTab("In Match").add(getChargingStation).withWidget(BuiltInWidgets.kComboBoxChooser);
                Shuffleboard.getTab("In Match").add(getDesiredHeight).withWidget(BuiltInWidgets.kComboBoxChooser);
                new Thread(() -> {
                        try {
                                Thread.sleep(1000);
                                alliance = DriverStation.getAlliance();
                                isRedAlliance = Alliance.Red == alliance && Alliance.Blue != alliance;
                                ;
                        } catch (Exception e) {
                        }
                }).start();
                intakeSubsystem = new MMIntakeSubsystem(this);

                leftTrigger = new Trigger(this::getLeftTriggerActive);
                rightTrigger = new Trigger(this::getRightTriggerActive);

                driverLeftPOV = new POVButton(driverJoystick, 270);
                driverRightPOV = new POVButton(driverJoystick, 90);
                driverUpPOV = new POVButton(driverJoystick, 0);
                driverDownPOV = new POVButton(driverJoystick, 180);

                // intakeSubsystem.setDefaultCommand(
                // new PositionHomeCmd(this));

                swerveSubsystem.setDefaultCommand(
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> navigationSubsystem.setClawPipeline(0)),
                                                new SwerveJoystickCmd(this,
                                                                () -> driveXAxis.getSquared(),
                                                                () -> driveYAxis.getSquared(),
                                                                () -> driveRAxis.getSquared(),
                                                                () -> false))
                // driverJoystick.getRawButton(
                // Constants.Driver.Button.overrideFieldCentricA)

                );
                configureBindings();
        }

        public DeliveryMethod selectDeliveryMethod() {// TODO revert back
                // if (intakeSubsystem.getBeamBreak()) {
                // if (gridHeight == 1) {
                // return DeliveryMethod.DeliverFloorCube;
                // }
                // return DeliveryMethod.DeliverMiddleCube;
                // }
                if (MMField.isCellCone(gridCell)) {
                        return DeliveryMethod.DeliverCone;
                }
                return DeliveryMethod.DeliverMiddleCube;
        }

        private void configureBindings() {
                leftTrigger.whileTrue(
                                new SequentialCommandGroup(
                                                new TargetPegLateralCmd(this, 1, () -> intakeSubsystem.getBeamBreak()),
                                                new TargetPegDriveCmd(this, 1, () -> intakeSubsystem.getBeamBreak(),
                                                                .25)));

                // driverLeftPOV.onTrue(
                // new InstantCommand(new ));

                // leftTrigger.whileTrue(new SequentialCommandGroup(
                // new TranslateAbsoluteCmd(this,
                // () -> MMField.getLeftDock(this::getIsRedAlliance), 2)
                // .until(navigationSubsystem::approachingLoadingDock),
                // // new TranslateAbsoluteCmd(swerveSubsystem,
                // // () -> MMField.getLeftDock(this::getIsRedAlliance), .25,
                // navigationSubsystem))
                // new DriveToBumperCmd(this, .5),

                // new TranslateAbsoluteCmd(this,
                // () -> MMField.getLeftDockRetractPoint(this::getIsRedAlliance),
                // 2)));

                // rightTrigger.whileTrue(new SequentialCommandGroup(
                // new TranslateAbsoluteCmd(this,
                // () -> MMField.getRightDock(this::getIsRedAlliance), 2)
                // .until(navigationSubsystem::approachingLoadingDock),
                // // new TranslateAbsoluteCmd(swerveSubsystem,
                // // () -> MMField.getLeftDock(this::getIsRedAlliance), .25,
                // navigationSubsystem))
                // new DriveToBumperCmd(this, .5),
                // new TranslateAbsoluteCmd(this,
                // () -> MMField.getRightDockRetractPoint(this::getIsRedAlliance),
                // 2)));
                // new JoystickButton(buttonBox1, 7)
                // .onTrue(new PositionHighPegCmd(this));

                // new JoystickButton(buttonBox1, 9)
                // .whileTrue(new PositionLowPegCmd(this));
                new JoystickButton(driverJoystick, 3)
                                .whileTrue(new LeftDockExtract(this));
                new JoystickButton(driverJoystick, 2)
                                .whileTrue(new RightDockExtract(this));
                new JoystickButton(driverJoystick, 5)
                                .whileTrue(new StartEndCommand(() -> intakeSubsystem.runIntake(),
                                                () -> intakeSubsystem.stopIntake()));

                new JoystickButton(driverJoystick, 6)
                                .whileTrue(new StartEndCommand(() -> intakeSubsystem.runOutTakeHigh(),
                                                () -> intakeSubsystem.stopIntake()));

                new JoystickButton(buttonBox1, 1)
                                .whileTrue(new PositionHomeCmd(this));
                new JoystickButton(buttonBox1, 2)
                                .whileTrue(new PositionLoadingCmd(this));
                new JoystickButton(buttonBox1, 3)
                                .whileTrue(new ManualDeliverCmd(this, () -> getManualDeliver()));
                // new JoystickButton(buttonBox1, 4)
                // .whileTrue(new PositionHighPegCmd(this));
                new JoystickButton(driverJoystick, 4)
                                .whileTrue(new AutoDeliveryCmd(this));
                driverLeftPOV.onTrue(new AdjustArmPoseCmd(() -> AdjustSelection.RotateIncrease, this));
                driverRightPOV.onTrue(new AdjustArmPoseCmd(() -> AdjustSelection.RotateDecrease, this));
                driverUpPOV.onTrue(new AdjustArmPoseCmd(() -> AdjustSelection.ExtendIncrease, this));
                driverDownPOV.onTrue(new AdjustArmPoseCmd(() -> AdjustSelection.ExtendDecrease, this));
                // new JoystickButton(buttonBox1, 5)
                // .whileTrue(new AutoDeliveryCmd(this));
                // new JoystickButton(buttonBox1, 5)
                // .onTrue(new SequentialCommandGroup(
                // new InstantCommand(() -> setStartingExtension()),
                // new InstantCommand(() -> intakeSubsystem
                // .setArmExtend((getStartingExtension() - .0254)))));
                // new JoystickButton(buttonBox1, 6)
                // .onTrue(new SequentialCommandGroup(
                // new InstantCommand(() -> setStartingExtension()),
                // new InstantCommand(() -> intakeSubsystem
                // .setArmExtend((getStartingExtension() + .0254)))));
                new JoystickButton(buttonBox1, 7)
                                .onTrue(new PositionGroundCmd(this));
                // new JoystickButton(driverJoystick, 5)
                // .onTrue(new SequentialCommandGroup(
                // new InstantCommand(() -> setStartingAngle()),
                // new InstantCommand(() -> intakeSubsystem
                // .setArmRotation((getStartingAngle() + .0494)))));
                // new JoystickButton(driverJoystick, 6)
                // .onTrue(new SequentialCommandGroup(
                // new InstantCommand(() -> setStartingAngle()),
                // new InstantCommand(() -> intakeSubsystem
                // .setArmRotation((getStartingAngle() - .0494)))));
                // new JoystickButton(buttonBox1, 6)
                // .whileTrue(new InstantCommand());
                // new JoystickButton(buttonBox1, 5)
                // .whileTrue(new TargetTagLateralCmd(this, 1, () -> false));
                // new JoystickButton(buttonBox1, 6)
                // .whileTrue(new TargetPegLateralCmd(this, 1, () -> false));
                // new JoystickButton(buttonBox1, 7)
                // .whileTrue(new PositionGroundCmd(this));

                // new JoystickButton(buttonBox1, 10)
                // .whileTrue(
                // new SequentialCommandGroup(new TargetPegCmd(this, 1, () -> false),
                // new DriveToBumperCmd(this, .5)));
                // new JoystickButton(buttonBox1, 3)
                // .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeDeliverLower()));
                // new JoystickButton(buttonBox1, 4)
                // .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeDeliverUpper()));
                new JoystickButton(buttonBox1, 9)
                                .onTrue(new GripReleaseCmd(this));
                new JoystickButton(buttonBox1, 8)
                                .onTrue(new GripGrabCmd(this));
                new JoystickButton(buttonBox1, 10)
                                .whileTrue(new LockedInCmd(this));

                // new JoystickButton(buttonBox1, Constants.Driver.Button.runIntake)
                // .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeFloor()));

                // intakeSubsystem.setIntakeFloor()
                // new JoystickButton(driverJoystick, Constants.Driver.Button.runIntake)
                // .whileTrue(new ParallelCommandGroup(
                // new RunIntakeUntilBrokenCmd(this),
                // new StartEndCommand(() -> intakeSubsystem.setIntakeDeliverLower(),
                // () -> intakeSubsystem.setIntakeTravel())));

                // new JoystickButton(driverJoystick, Constants.Driver.Button.runIntake)
                // .whileTrue(new StartEndCommand(() -> intakeSubsystem.runIntake(), () ->
                // intakeSubsystem.stopIntake()));

                // new JoystickButton(driverJoystick, Constants.Driver.Button.runIntake)
                // .whileTrue(new StartEndCommand(() -> intakeSubsystem.runIntake(), () ->
                // intakeSubsystem.stopIntake()));

                // new JoystickButton(driverJoystick, 6)
                // .whileTrue(new StartEndCommand(() -> intakeSubsystem.runOutTake(),
                // () -> intakeSubsystem.stopIntake()));

                // .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeDeliverUpper()));

                // new JoystickButton(buttonBox1, 10)
                // .whileTrue(new InstantCommand(() -> intakeSubsystem.setArmRotationNeg()));

                new JoystickButton(driverJoystick, Constants.Driver.Button.resetNavxA)
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> navigationSubsystem
                                                                .zeroHeading(() -> getIsRedAlliance())),
                                                new InstantCommand(() -> navigationSubsystem
                                                                .resetOdometry(
                                                                                navigationSubsystem.getLimelightPose()
                                                                // new Pose2d(3.3, -4, new Rotation2d())
                                                                ))));

                // new JoystickButton(buttonBox1, 2)
                // .onTrue(new InstantCommand(() -> intakeSubsystem.setIntakeFloor()));

                // new JoystickButton(driverJoystick, Constants.Driver.Button.testPeg)
                // .onTrue(
                // new DriveToBumperCmd(navigationSubsystem, swerveSubsystem, .5));
                // rightTrigger.whileTrue(new
                // AutoDeliveryCmd(this).unless(this::isNotGridCellSelected));
                rightTrigger.whileTrue(
                                new SequentialCommandGroup(
                                                new TargetTagLateralCmd(this, 1, () -> intakeSubsystem.getBeamBreak()),
                                                new TargetTagDriveCmd(this, 1, () -> intakeSubsystem.getBeamBreak(),
                                                                .25)));
                // new JoystickButton(driverJoystick, rightTrigger)
                // .whileTrue(new AutoDeliveryCmd(this).unless(this::isNotGridCellSelected));
                // swerveSubsystem, navigationSubsystem,
                // this::getIsRedAlliance,
                // this::getGridCell,
                // // this::selectDeliveryMethod
                // new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.testTurnPeg)
                // .whileTrue(
                // new TargetPegCmd(this, gridCell,
                // intakeSubsystem::getBeamBreak));
                // new JoystickButton(driverJoystick, Constants.Driver.Button.runIntake)
                // // .whileTrue(
                // // new StartEndCommand(
                // // () -> intakeSubsystem.runIntake(),
                // // // Stop driving at the end of the command
                // // () -> intakeSubsystem.stopIntake(),
                // // // Requires the drive subsystem
                // // intakeSubsystem));
                // .onTrue(
                // new PickUpCubeCmd(this));
                // new JoystickButton(driverJoystick, Constants.Driver.Button.runOutTake)
                // .whileTrue(
                // new StartEndCommand(
                // () -> intakeSubsystem.runOutTake(),
                // // Stop driving at the end of the command
                // () -> intakeSubsystem.stopIntake(),
                // // Requires the drive subsystem
                // intakeSubsystem));
                // .onTrue(
                // new DeliverCubeCmd(this, () -> false));

                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.row1)
                                .onTrue(new InstantCommand(() -> selectCell(gridCell, 1)));

                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.row2)
                                .onTrue(new InstantCommand(() -> selectCell(gridCell, 2)));

                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.row3)
                                .onTrue(new InstantCommand(() -> selectCell(gridCell, 3)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col1)
                                .onTrue(new InstantCommand(() -> selectCell(1, gridHeight)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col2)
                                .onTrue(new InstantCommand(() -> selectCell(2, gridHeight)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col3)
                                .onTrue(new InstantCommand(() -> selectCell(3, gridHeight)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col4)
                                .onTrue(new InstantCommand(() -> selectCell(4, gridHeight)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col5)
                                .onTrue(new InstantCommand(() -> selectCell(5, gridHeight)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col6)
                                .onTrue(new InstantCommand(() -> selectCell(6, gridHeight)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col7)
                                .onTrue(new InstantCommand(() -> selectCell(7, gridHeight)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col8)
                                .onTrue(new InstantCommand(() -> selectCell(8, gridHeight)));
                new JoystickButton(buttonBox2, Constants.ButtonBox2.Button.col9)
                                .onTrue(new InstantCommand(() -> selectCell(9, gridHeight)));

                // new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroup1)
                // .onTrue(new InstantCommand(() -> selectCell(gridHeight, 1, gridGroupCell)));
                // new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroup2)
                // .onTrue(new InstantCommand(() -> selectCell(gridHeight, 2, gridGroupCell)));
                // new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroup3)
                // .onTrue(new InstantCommand(() -> selectCell(gridHeight, 3, gridGroupCell)));
                // new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupCell1)
                // .onTrue(new InstantCommand(() -> selectCell(gridHeight, gridGroup, 1)));
                // new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupCell2)
                // .onTrue(new InstantCommand(() -> selectCell(gridHeight, gridGroup, 2)));
                // new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupCell3)
                // .onTrue(new InstantCommand(() -> selectCell(gridHeight, gridGroup, 3)));
                // new JoystickButton(buttonBox1,
                // Constants.ButtonBox1.Button.gridGroupHeightLow)
                // .onTrue(new InstantCommand(() -> selectCell(1, gridGroup, gridGroupCell)));
                // new JoystickButton(buttonBox1,
                // Constants.ButtonBox1.Button.gridGroupHeightMed)
                // .onTrue(new InstantCommand(() -> selectCell(2, gridGroup, gridGroupCell)));
                // new JoystickButton(buttonBox1,
                // Constants.ButtonBox1.Button.gridGroupHeightHigh)
                // .onTrue(new InstantCommand(() -> selectCell(3, gridGroup, gridGroupCell)));
        }

        public Command getAutonomousCommand() {
                // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 1.7)
                // .setKinematics(Constants.Chassis.kinematics);
                // // return Commands.print("No autonomous command configured");
                // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                // new Pose2d(0, 0, new Rotation2d()),
                // List.of(
                // new Translation2d(1, 0),
                // new Translation2d(1, -1),
                // new Translation2d(2.4, -1),
                // new Translation2d(2.4, 0.8),
                // new Translation2d(1, 0.8),
                // new Translation2d(1, 0)),
                // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                // trajectoryConfig);

                // PIDController xController = new PIDController(.0, 0, 0);
                // PIDController yController = new PIDController(10.0, 0, 0);
                // ProfiledPIDController thetaController = new ProfiledPIDController(10.0, 0, 0,
                // Constants.thetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // trajectory,
                // navigationSubsystem::getPose,
                // Constants.Chassis.kinematics,
                // xController,
                // yController,
                // thetaController,
                // swerveSubsystem::setModuleStates,
                // swerveSubsystem);
                switch (getDesiredAuto.getSelected()) {
                        case TwoBallAuto:
                                return new TwoBallAutoCmd(this);
                        case OneConeAuto:
                                return new OneConeAutoCmd(this, () -> autoPosition, () -> isOverChargingStation);
                        default:
                                return new OneBallAutoCmd(this, () -> autoPosition, () -> isOverChargingStation);

                }

        }

        // public void selectCell(int gridHeight, int gridGroup, int gridGroupCell) {
        // gridCell = gridGroup * 3 - 3 + gridGroupCell;
        // this.gridGroup = gridGroup;
        // this.gridGroupCell = gridGroupCell;
        // this.gridHeight = gridHeight;
        // gridCellEntry.setString("Grid Group: " + gridGroup + "Grid GroupCell: " +
        // gridGroupCell
        // + " Grid Height: " + gridHeight + " Grid Cell: " + gridCell);
        // }

        public void selectCell(int column, int row) {
                gridCell = column;
                gridHeight = row;
                gridCellEntry.setString("Grid Cell: " + gridCell + "Grid Height: " + gridHeight);
        }

        public boolean getIsRedAlliance() {
                return isRedAlliance;
        }

        public void setAutoPosition() {
                autoPosition = MMField.getAutoCellPose(getAutoStartPose.getSelected(), isRedAlliance);
        }

        public void setChargingStation() {
                isOverChargingStation = getChargingStation.getSelected();
        }

        public void incAngle() {
                manualAngle++;
        }

        public double getManualAngle() {
                return manualAngle;
        }

        public void decAngle() {
                manualAngle--;
        }

        public void setStartingExtension() {
                startingExtension = intakeSubsystem.getArmExtend();
        }

        public double getStartingAngle() {
                return startingAngle;
        }

        public double getStartingExtension() {
                return startingExtension;
        }

        public void setStartingAngle() {
                startingAngle = intakeSubsystem.getArmRotate();
        }

        public int getGridCell() {
                return gridCell;
        }

        public int getGridHeight() {
                return gridHeight;
        }

        public boolean isGridCellSelected() {
                return gridCell >= 1 && gridCell <= 9;
        }

        public boolean isNotGridCellSelected() {
                return !(isGridCellSelected());
        }

        public void clearSelectedCell() {
                gridCell = 0;
        }

        public boolean getDriverUpPOV() {
                return driverJoystick.getPOV() == 0;
        }

        public boolean getDriverDownPOV() {
                return driverJoystick.getPOV() == 180;
        }

        public boolean getDriverLeftPOV() {
                return driverJoystick.getPOV() == 270;
        }

        public boolean getDriverRightPOV() {
                return driverJoystick.getPOV() == 90;
        }

        public boolean getLeftTriggerActive() {
                return driverJoystick.getRawAxis(Constants.Driver.Axis.lt) > .5;
        }

        public boolean getRightTriggerActive() {
                return driverJoystick.getRawAxis(Constants.Driver.Axis.rt) > .5;
        }

        public int buttonCellSelect() {
                int column = 0;
                int row = 0;
                for (int col = 1; col < 10; col++) {
                        if (buttonBox2.getRawButtonPressed(col)) {
                                column = col;
                        }
                }
                for (int r = 10; r < 13; r++) {
                        if (buttonBox2.getRawButtonPressed(r)) {
                                row = r;
                        }
                }
                if (row == 0 || column == 0) {
                        return 0;
                }
                row -= 10;
                return (row * 9) + column;
        }

        public ManualDeliveryMethod getManualDeliver() {
                if (intakeSubsystem.getBeamBreak()) {
                        switch (gridHeight) {
                                case 1:
                                        return ManualDeliveryMethod.CubeLowNode;
                                case 2:
                                        return ManualDeliveryMethod.CubeMiddleNode;
                                case 3:
                                        return ManualDeliveryMethod.CubeHighNode;
                        }

                } else {
                        switch (gridHeight) {
                                case 1:
                                        return ManualDeliveryMethod.ClawLowNode;
                                case 2:
                                        return ManualDeliveryMethod.ClawMiddleNode;
                                case 3:
                                        return ManualDeliveryMethod.ClawHighNode;
                        }
                }
                return ManualDeliveryMethod.NoneSelected;
        }

}
// create a mega command to select choose whether we are delivering or if we are
// using the cell selection to do semi-automatic;