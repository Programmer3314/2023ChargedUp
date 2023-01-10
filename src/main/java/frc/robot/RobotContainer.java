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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMJoystickAxis;

public class RobotContainer {

  private final MMSwerveSubsystem swerveSubsystem = new MMSwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(Constants.DriverController);
  // TODO: Slow these down... They are in METERS per Second not feet. 3 meters per
  // second is pretty fast for initial testing
  private final MMJoystickAxis driveXAxis = new MMJoystickAxis(Constants.DriverController, Constants.driveXAxis, 0.01,
      -1.3);
  private final MMJoystickAxis driveYAxis = new MMJoystickAxis(Constants.DriverController, Constants.driveYAxis, 0.01,
      -1.3);
  private final MMJoystickAxis driveRAxis = new MMJoystickAxis(Constants.DriverController, Constants.driveRAxis, 0.01,
      -(Math.PI / 2.0));

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> driveXAxis.getSquared(),
        () -> driveYAxis.getSquared(),
        () -> driveRAxis.getSquared(),
        () -> driverJoystick.getRawButton(Constants.setFieldOriented)));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, 2)
        .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1, 1).setKinematics(Constants.driveKinematics);
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

    PIDController xController = new PIDController(0.8, 0, 0);
    PIDController yController = new PIDController(0.8, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(0.5, 0, 0, Constants.thetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory,
        swerveSubsystem::getPose,
        Constants.driveKinematics,
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
