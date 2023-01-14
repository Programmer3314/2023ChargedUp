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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LockedInCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TranslateAbsoluteCmd;
import frc.robot.commands.TranslateRelativeCmd;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMJoystickAxis;

public class RobotContainer {

        private final MMSwerveSubsystem swerveSubsystem = new MMSwerveSubsystem();

        private final Joystick driverJoystick = new Joystick(Constants.Driver.Controller);
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
        private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
        private final NetworkTable limelight = inst.getTable("limelight");

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
                                () -> driveXAxis.getSquared(),
                                () -> driveYAxis.getSquared(),
                                () -> driveRAxis.getSquared(),
                                () -> driverJoystick.getRawButton(Constants.Driver.Button.overrideFieldCentric)));

                configureBindings();
        }

        private void configureBindings() {
                new JoystickButton(driverJoystick, Constants.Driver.Button.resetNavx)
                                .onTrue(new SequentialCommandGroup(
                                                new InstantCommand(() -> swerveSubsystem.zeroHeading()),
                                                new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d()))));
                new JoystickButton(driverJoystick, Constants.Driver.Button.lockIn)
                                .onTrue(new SequentialCommandGroup(
                                                new TranslateAbsoluteCmd(swerveSubsystem, new Translation2d(3, 1), 1.0),
                                                new LockedInCmd(swerveSubsystem)));
                new JoystickButton(driverJoystick, Constants.Driver.Button.trackAprilTag)
                                .whileTrue(new SwerveJoystickCmd(swerveSubsystem,
                                                () -> driveXAxis.getSquared(),
                                                () -> driveYAxis.getSquared(),
                                                () -> limelight.getEntry("tx").getDouble(0) * .15,
                                                () -> driverJoystick.getRawButton(
                                                                Constants.Driver.Button.overrideFieldCentric)));
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
                                swerveSubsystem::getPose,
                                Constants.Chassis.kinematics,
                                xController,
                                yController,
                                thetaController,
                                swerveSubsystem::setModuleStates,
                                swerveSubsystem);

                return new SequentialCommandGroup(
                                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                                swerveControllerCommand,
                                new InstantCommand(() -> swerveSubsystem.stopModules()));
        }
}
