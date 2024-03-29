// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DeliverCubeCmd extends SequentialCommandGroup {
    public DeliverCubeCmd(RobotContainer rc, BooleanSupplier isLow) {
        addCommands(
                new PositionHomeCmd(rc),
                Commands.parallel(
                        Commands.either(
                                new InstantCommand(rc.intakeSubsystem::runOutTakeSlow),
                                new InstantCommand(rc.intakeSubsystem::runOutTakeHigh),
                                isLow),
                        Commands.waitSeconds(1)),
                new InstantCommand(rc.intakeSubsystem::stopIntake));
    }
}
