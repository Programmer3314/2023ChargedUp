// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// TODO: Add arm movements

/** Add your docs here. */
public class DeliverConeCmd extends SequentialCommandGroup {

    public DeliverConeCmd(double maxRotationSpeed,
            RobotContainer rc) {
        addCommands(
                Commands.race(
                        new TargetPegCmd(rc, 2,
                                rc.intakeSubsystem::getBeamBreak),
                        new WaitToDeliverCmd(2)),
                new DriveToBumperCmd(rc, .5),
                new DriveToGridAlleyCmd(rc, ()-> true)
                );
    }
}
