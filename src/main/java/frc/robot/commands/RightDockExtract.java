// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.utility.MMField;

/** Add your docs here. */
public class RightDockExtract extends SequentialCommandGroup {
    public RightDockExtract(RobotContainer rc) {
        addCommands(
                new ParallelCommandGroup(
                        new GripReleaseCmd(rc),
                        new TranslateAbsoluteCmd(rc,
                                () -> MMField.getRightDockRetractPoint(() -> rc.getIsRedAlliance()), 1)),
                new ParallelCommandGroup(new DriveToBumperCmd(rc, .5),
                        new PositionLoadingCmd(rc)),
                new GripGrabCmd(rc),
                new TranslateAbsoluteCmd(rc, () -> MMField.getRightDock(() -> rc.getIsRedAlliance()), 1));
    }
}
