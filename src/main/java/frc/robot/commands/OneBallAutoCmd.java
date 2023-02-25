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
public class OneBallAutoCmd extends SequentialCommandGroup {
    public OneBallAutoCmd(RobotContainer rc) {
        addCommands(
                new GripReleaseCmd(rc),
                new DriveToCellCmd(rc,
                        () -> 4,
                        rc::getIsRedAlliance,
                        1, rc.navigationSubsystem),
                new CompleteRampSequenceCmd(rc));
    }
}