// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.MMSwerveSubsystem;
import frc.robot.utility.MMJoystickAxis;

public class RobotContainer {

  private final MMSwerveSubsystem swerveSubsystem = new MMSwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(Constants.DriverController);
  private final MMJoystickAxis driveXAxis = new MMJoystickAxis(Constants.DriverController, Constants.driveXAxis, 0.01,
      -3);
  private final MMJoystickAxis driveYAxis = new MMJoystickAxis(Constants.DriverController, Constants.driveYAxis, 0.01,
      -3);
  private final MMJoystickAxis driveRAxis = new MMJoystickAxis(Constants.DriverController, Constants.driveRAxis, 0.01,
      Math.PI / 2);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> driveXAxis.getSquared(),
        () -> driveYAxis.getSquared(),
        () -> driveRAxis.getSquared(),
        () -> driverJoystick.getRawButton(Constants.setFieldOriented)));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, Constants.setFieldOriented)
        .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
