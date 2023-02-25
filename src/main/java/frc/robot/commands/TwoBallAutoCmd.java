// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class TwoBallAutoCmd extends SequentialCommandGroup {
    public TwoBallAutoCmd(RobotContainer rc) {
        addCommands(
                new GripReleaseCmd(rc));
        // new TranslateAbsoluteCmd(rc, () -> new Pose2d(new Translation2d(13.3, -2.9),
        // new Rotation2d()), 1))

    }
}
