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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoDeliveryCmd;
import frc.robot.commands.DriveToBumperCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TranslateAbsoluteCmd;
import frc.robot.subsystems.MMIntakeSubsystem;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMField;
import frc.robot.utility.MMJoystickAxis;

public class RobotContainer {
    private ShuffleboardTab tab = Shuffleboard.getTab("In Match");

    // private final SendableChooser<Integer> getDesiredCell = new
    // SendableChooser<>();

    private static Alliance alliance;
    private static boolean isRedAlliance = true;

    public final MMSwerveSubsystem swerveSubsystem = new MMSwerveSubsystem();
    public final MMNavigationSubsystem navigationSubsystem = new MMNavigationSubsystem(swerveSubsystem);

    private final Joystick driverJoystick = new Joystick(Constants.Driver.Controller);
    private final Joystick buttonBox1 = new Joystick(Constants.ButtonBox1.button);
    private final MMJoystickAxis driveXAxis = new MMJoystickAxis(Constants.Driver.Controller,
            Constants.Driver.Axis.x,
            0.01,
            -1.3);
    private final MMJoystickAxis driveYAxis = new MMJoystickAxis(Constants.Driver.Controller,
            Constants.Driver.Axis.y,
            0.01,
            -1.3);
    private final MMJoystickAxis driveRAxis = new MMJoystickAxis(Constants.Driver.Controller,
            Constants.Driver.Axis.r,
            0.05,
            -(Math.PI / 2.0));
    private int gridHeight;
    private int gridGroup;
    private int gridGroupCell;
    private int gridCell;
    private GenericEntry gridCellEntry = tab.add("Grid Cell: ", "None").getEntry();
    private Trigger leftTrigger;
    private Trigger rightTrigger;
    private final MMIntakeSubsystem intakeSubsystem;

    public RobotContainer() {

        // getDesiredCell.setDefaultOption("None Selected", 0);
        // for (int i = 1; i < 10; i++) {
        // getDesiredCell.addOption("Cell: " + i, i);
        // }

        // Shuffleboard.getTab("In
        // Match").add(getDesiredCell).withWidget(BuiltInWidgets.kComboBoxChooser);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                alliance = DriverStation.getAlliance();
                isRedAlliance = Alliance.Red == alliance && Alliance.Blue != alliance;
                ;
            } catch (Exception e) {
            }
        }).start();
        intakeSubsystem=new MMIntakeSubsystem();

        leftTrigger = new Trigger(this::getLeftTriggerActive);
        rightTrigger = new Trigger(this::getRightTriggerActive);

        swerveSubsystem.setDefaultCommand(
                new SequentialCommandGroup(
                        new InstantCommand(() -> navigationSubsystem.setPipeline(0)),
                        new SwerveJoystickCmd(swerveSubsystem,
                                () -> driveXAxis.getSquared(),
                                () -> driveYAxis.getSquared(),
                                () -> driveRAxis.getSquared(),
                                () -> driverJoystick.getRawButton(
                                        Constants.Driver.Button.overrideFieldCentricA),
                                navigationSubsystem))

        );
        configureBindings();
    }

    public DeliveryMethod selectDeliveryMethod() {
        if (MMField.isCellCone(gridCell)) {
            return DeliveryMethod.DeliverCone;
        }
        return DeliveryMethod.DeliverHighCube;
    }

    private void configureBindings() {
        leftTrigger.whileTrue(new SequentialCommandGroup(
                new TranslateAbsoluteCmd(swerveSubsystem, () -> MMField.getLeftDock(this::getIsRedAlliance), 2,
                        navigationSubsystem)
                        .until(navigationSubsystem::approachingLoadingDock),
                // new TranslateAbsoluteCmd(swerveSubsystem,
                // () -> MMField.getLeftDock(this::getIsRedAlliance), .25, navigationSubsystem))
                new DriveToBumperCmd(navigationSubsystem, swerveSubsystem, .5),
                new TranslateAbsoluteCmd(swerveSubsystem, () -> MMField.getLeftDockRetractPoint(this::getIsRedAlliance),
                        2,
                        navigationSubsystem)));

        rightTrigger.whileTrue(new SequentialCommandGroup(
                new TranslateAbsoluteCmd(swerveSubsystem, () -> MMField.getRightDock(this::getIsRedAlliance), 2,
                        navigationSubsystem)
                        .until(navigationSubsystem::approachingLoadingDock),
                // new TranslateAbsoluteCmd(swerveSubsystem,
                // () -> MMField.getLeftDock(this::getIsRedAlliance), .25, navigationSubsystem))
                new DriveToBumperCmd(navigationSubsystem, swerveSubsystem, .5),
                new TranslateAbsoluteCmd(swerveSubsystem,
                        () -> MMField.getRightDockRetractPoint(this::getIsRedAlliance),
                        2,
                        navigationSubsystem)));

        new JoystickButton(driverJoystick, Constants.Driver.Button.resetNavxB)
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> navigationSubsystem.zeroHeading()),
                        new InstantCommand(() -> navigationSubsystem
                                .resetOdometry(
                                        navigationSubsystem.getLimelightPose()
                                // new Pose2d(3.3, -4, new Rotation2d())
                                ))));

        new JoystickButton(driverJoystick, Constants.Driver.Button.testPeg)
                .onTrue(
                        new DriveToBumperCmd(navigationSubsystem, swerveSubsystem, .5));

        // new JoystickButton(driverJoystick, 6)
        //         .whileTrue(new AutoDeliveryCmd(this).unless(this::isNotGridCellSelected));
        // swerveSubsystem, navigationSubsystem,
        // this::getIsRedAlliance,
        // this::getGridCell,
        // this::selectDeliveryMethod
        new JoystickButton(driverJoystick, Constants.Driver.Button.runIntake)
        .whileTrue(new InstantCommand(()-> intakeSubsystem.runIntake()));

        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroup1)
                .onTrue(new InstantCommand(() -> selectCell(gridHeight, 1, gridGroupCell)));
        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroup2)
                .onTrue(new InstantCommand(() -> selectCell(gridHeight, 2, gridGroupCell)));
        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroup3)
                .onTrue(new InstantCommand(() -> selectCell(gridHeight, 3, gridGroupCell)));
        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupCell1)
                .onTrue(new InstantCommand(() -> selectCell(gridHeight, gridGroup, 1)));
        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupCell2)
                .onTrue(new InstantCommand(() -> selectCell(gridHeight, gridGroup, 2)));
        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupCell3)
                .onTrue(new InstantCommand(() -> selectCell(gridHeight, gridGroup, 3)));
        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupHeightLow)
                .onTrue(new InstantCommand(() -> selectCell(1, gridGroup, gridGroupCell)));
        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupHeightMed)
                .onTrue(new InstantCommand(() -> selectCell(2, gridGroup, gridGroupCell)));
        new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.gridGroupHeightHigh)
                .onTrue(new InstantCommand(() -> selectCell(3, gridGroup, gridGroupCell)));
    }

    public Command getAutonomousCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(2, 1.7)
                .setKinematics(Constants.Chassis.kinematics);
        // return Commands.print("No autonomous command configured");
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d()),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1),
                        new Translation2d(2.4, -1),
                        new Translation2d(2.4, 0.8),
                        new Translation2d(1, 0.8),
                        new Translation2d(1, 0)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        PIDController xController = new PIDController(10.0, 0, 0);
        PIDController yController = new PIDController(10.0, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(10.0, 0, 0,
                Constants.thetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                navigationSubsystem::getPose,
                Constants.Chassis.kinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> navigationSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public void selectCell(int gridHeight, int gridGroup, int gridGroupCell) {
        gridCell = gridGroup * 3 - 3 + gridGroupCell;
        this.gridGroup = gridGroup;
        this.gridGroupCell = gridGroupCell;
        this.gridHeight = gridHeight;
        gridCellEntry.setString("Grid Group: " + gridGroup + "Grid GroupCell: " + gridGroupCell
                + " Grid Height: " + gridHeight + " Grid Cell: " + gridCell);
    }

    public boolean getIsRedAlliance() {
        return isRedAlliance;
    }

    public int getGridCell() {
        return gridCell;
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

    public boolean getLeftTriggerActive() {
        return driverJoystick.getRawAxis(Constants.Driver.Axis.lt) > .5;
    }

    public boolean getRightTriggerActive() {
        return driverJoystick.getRawAxis(Constants.Driver.Axis.rt) > .5;
    }

}