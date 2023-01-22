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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveAbsoluteDistance;
import frc.robot.commands.DriveToCell;
import frc.robot.commands.DriveToRampCmd;
import frc.robot.commands.LockedInCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TargetPegCmd;
import frc.robot.commands.TargetTagCmd;
import frc.robot.commands.TranslateAbsoluteCmd;
import frc.robot.commands.TranslateRelativeCmd;
import frc.robot.commands.WaitToDeliverCmd;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMJoystickAxis;

public class RobotContainer {

        private final SendableChooser<Integer> getDesiredCell = new SendableChooser<>();

        private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
        private static Alliance alliance;
        private static boolean isRedAlliance;

        private final MMSwerveSubsystem swerveSubsystem = new MMSwerveSubsystem();
        private final MMNavigationSubsystem navigationSubsystem = new MMNavigationSubsystem(swerveSubsystem);

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

        public RobotContainer() {

                getDesiredCell.setDefaultOption("None Selected", 0);
                for (int i = 1; i < 10; i++) {
                        getDesiredCell.addOption("Cell: " + i, i);
                }
                
                Shuffleboard.getTab("In Match").add(getDesiredCell).withWidget(BuiltInWidgets.kComboBoxChooser);
                
                alliance = DriverStation.getAlliance();
                isRedAlliance = Alliance.Red == alliance;

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

        private void configureBindings() {
                new JoystickButton(driverJoystick, Constants.Driver.Button.resetNavxB)
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> navigationSubsystem.zeroHeading()),
                                                new InstantCommand(() -> navigationSubsystem
                                                                .resetOdometry(
                                                                                navigationSubsystem.getLimelightPose()
                                                                // new Pose2d(3.3, -4, new Rotation2d())
                                                                ))));
                new JoystickButton(driverJoystick, Constants.Driver.Button.targetTagY)
                                // .onTrue(new ParallelCommandGroup(
                                // new TargetTagCmd(swerveSubsystem, 2, navigationSubsystem, 1)),
                                // new WaitRO);
                                // ))
                                .onTrue(new SequentialCommandGroup(

                                                // new InstantCommand(() -> navigationSubsystem.changePipeline(0)),
                                                // new TranslateAbsoluteCmd(swerveSubsystem,
                                                // new Pose2d(3.69, -3.93, new Rotation2d()), 1,
                                                // navigationSubsystem),
                                                new TranslateAbsoluteCmd(swerveSubsystem,
                                                                new Pose2d(6.1, -3.5, new Rotation2d(0)),
                                                                1, navigationSubsystem),
                                                new TranslateAbsoluteCmd(swerveSubsystem,
                                                                new Pose2d(6, -2.93, new Rotation2d(0)),
                                                                1, navigationSubsystem),
                                                // new LockedInCmd(swerveSubsystem)),
                                                // new InstantCommand(() -> navigationSubsystem.changePipeline(1))));
                                                new ParallelRaceGroup(
                                                                new TargetTagCmd(swerveSubsystem, 2,
                                                                                navigationSubsystem),
                                                                new WaitToDeliverCmd(30))));

                new JoystickButton(driverJoystick, Constants.Driver.Button.targetPegX)
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> navigationSubsystem.setPipeline(0)),
                                                // new TranslateAbsoluteCmd(swerveSubsystem,
                                                // new Pose2d(3.69, -3.93, new Rotation2d()),
                                                // 1, navigationSubsystem),
                                                new TranslateAbsoluteCmd(swerveSubsystem,
                                                                new Pose2d(6, -3.5, new Rotation2d(0)),
                                                                1, navigationSubsystem),
                                                new TranslateAbsoluteCmd(swerveSubsystem,
                                                                new Pose2d(6, -2.4, new Rotation2d(0)),
                                                                1, navigationSubsystem),
                                                // new LockedInCmd(swerveSubsystem)),
                                                new ParallelRaceGroup(
                                                                new TargetPegCmd(swerveSubsystem, 2,
                                                                                navigationSubsystem),
                                                                new WaitToDeliverCmd(30))));
                // new TargetPegCmd(swerveSubsystem, 2, navigationSubsystem, 1)));
                new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.changePipeline)
                                .whileTrue(new TargetTagCmd(swerveSubsystem, 2, navigationSubsystem));

                new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.autoDriveToRamp)
                                .onTrue(new SequentialCommandGroup(
                                                new DriveToRampCmd(swerveSubsystem, navigationSubsystem, .9),
                                                new DriveAbsoluteDistance(swerveSubsystem, new Translation2d(-.75, 0),
                                                                .25, navigationSubsystem),
                                                // new TranslateRelativeCmd(swerveSubsystem, new Translation2d(0.25,0),
                                                // 0.5, navigationSubsystem),
                                                new LockedInCmd(swerveSubsystem)));

                new JoystickButton(buttonBox1, Constants.ButtonBox1.Button.driveRelative)
                                .onTrue(
                                                new DriveToCell(swerveSubsystem,
                                                                () -> getDesiredCell.getSelected(),
                                                                navigationSubsystem,
                                                                isRedAlliance,
                                                                1));
                // new JoystickButton(driverJoystick, Constants.Driver.Button.trackAprilTag)
                // .whileTrue(new SwerveJoystickCmd(swerveSubsystem,
                // () -> driveXAxis.getSquared(),
                // () -> driveYAxis.getSquared(),
                // () -> limelight.getEntry("tx").getDouble(0) * .15,
                // () -> driverJoystick.getRawButton(
                // Constants.Driver.Button.overrideFieldCentric),
                // navigationSubsystem));
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

}