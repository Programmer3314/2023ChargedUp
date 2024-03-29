// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;


/** Add your docs here. */
public class DeliverCubeMiddleCmd extends SequentialCommandGroup {
    public DeliverCubeMiddleCmd(double maxRotationSpeed, RobotContainer rc) {
        addCommands(
                new DriveToGridAlleyCmd(rc, () -> false),
                new TargetTagCmd(rc, 2,
                        rc.intakeSubsystem::getBeamBreak),
                new DriveToBumperCmd(rc, .5),
                new DeliverCubeCmd(rc, () -> false),
                new DriveToGridAlleyCmd(rc, () -> true));

    }

}
