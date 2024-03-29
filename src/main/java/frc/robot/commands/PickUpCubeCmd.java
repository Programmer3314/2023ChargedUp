// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class PickUpCubeCmd extends SequentialCommandGroup {
    public PickUpCubeCmd(RobotContainer rc) {
        addCommands(
                new PositionCubePickUpCmd(rc),
                new RunIntakeUntilBrokenCmd(rc),
                new PositionHomeCmd(rc));
    }
}
