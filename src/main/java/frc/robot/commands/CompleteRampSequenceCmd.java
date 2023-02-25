// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class CompleteRampSequenceCmd extends SequentialCommandGroup {
    public CompleteRampSequenceCmd(RobotContainer rc) {
        addCommands(
                new DriveToRampCmd(rc, -.8),
                new TranslateRelativeCmd(rc,
                        new Pose2d(new Translation2d(-1, 0), new Rotation2d()),
                        .5),
                new LockedInCmd(rc));
    }
}
