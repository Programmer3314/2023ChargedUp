// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MMIntakeSubsystem;

/** Add your docs here. */
public class DeliverCubeCmd extends SequentialCommandGroup {
    public DeliverCubeCmd(MMIntakeSubsystem intakeSubsystem, BooleanSupplier isLow) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(intakeSubsystem::setIntakeTravel),
                        new InstantCommand(intakeSubsystem::setIntakeDeliverLower)),
                Commands.parallel(
                        Commands.either(
                                new InstantCommand(intakeSubsystem::runOutTakeSlow),
                                new InstantCommand(intakeSubsystem::runOutTake), isLow),
                        Commands.waitSeconds(1)),
                new InstantCommand(intakeSubsystem::stopIntake));
    }
}
