// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AdjustSelection;
import frc.robot.ManualDeliveryMethod;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class RunOutTakeCmd extends SequentialCommandGroup {
    public RunOutTakeCmd(RobotContainer rc, Supplier<Object> method) {
        addCommands(
                Commands.select(Map.ofEntries(
                        Map.entry(ManualDeliveryMethod.CubeLowNode,
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new InstantCommand(
                                                        () -> rc.intakeSubsystem.setIntakeTravel()),
                                                Commands.waitSeconds(.5)),
                                        Commands.race(
                                                Commands.waitSeconds(0.5),
                                                new InstantCommand(() -> rc.intakeSubsystem.runOutTakeSlow())),
                                        new InstantCommand(() -> rc.intakeSubsystem.stopIntake()))),
                        Map.entry(ManualDeliveryMethod.CubeMiddleNode,
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new InstantCommand(
                                                        () -> rc.intakeSubsystem.setIntakeDeliverLower()),
                                                Commands.waitSeconds(.5)),
                                        Commands.race(
                                                Commands.waitSeconds(0.5),
                                                new InstantCommand(
                                                        () -> rc.intakeSubsystem.runOutTakeMiddle())),
                                        new InstantCommand(() -> rc.intakeSubsystem.stopIntake()))),
                        Map.entry(ManualDeliveryMethod.CubeHighNode,
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new InstantCommand(
                                                        () -> rc.intakeSubsystem.setIntakeDeliverUpper()),
                                                Commands.waitSeconds(.5)),
                                        Commands.race(
                                                Commands.waitSeconds(0.5),
                                                new InstantCommand(
                                                        () -> rc.intakeSubsystem.runOutTakeMiddle())),
                                        new InstantCommand(() -> rc.intakeSubsystem.stopIntake())))),
                        method));
    }

}
