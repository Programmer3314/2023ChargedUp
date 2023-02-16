// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DeliverCubeHighCmd extends SequentialCommandGroup {

    public DeliverCubeHighCmd(RobotContainer rc) {
        addCommands(new TargetTagCmd(rc, 2,
                rc.intakeSubsystem::getBeamBreak),
                new ParallelCommandGroup(
                        new DriveToBumperCmd(rc, 1),
                        new PositionHighPegCmd(rc)
                ),
                new GripReleaseCmd(rc),
                new DriveToGridAlleyCmd(rc, () -> true));

    }
}
