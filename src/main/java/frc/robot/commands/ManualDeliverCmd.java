// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.Instant;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AdjustSelection;
import frc.robot.ManualDeliveryMethod;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ManualDeliverCmd extends SequentialCommandGroup {
        public ManualDeliverCmd(RobotContainer rc, Supplier<Object> method) {

                addCommands(
                                Commands.select(Map.ofEntries(
                                                Map.entry(ManualDeliveryMethod.CubeLowNode,
                                                                new SequentialCommandGroup(
                                                                                new ParallelCommandGroup(
                                                                                                new InstantCommand(
                                                                                                                () -> rc.intakeSubsystem
                                                                                                                                .setIntakeTravel()),
                                                                                                Commands.waitSeconds(
                                                                                                                .5)),

                                                                                new SpeedCubeDeliverCmd(rc, 50, 0.2))),
                                                Map.entry(ManualDeliveryMethod.CubeMiddleNode,
                                                                new SequentialCommandGroup(
                                                                                new ParallelCommandGroup(
                                                                                                new InstantCommand(
                                                                                                                () -> rc.intakeSubsystem
                                                                                                                                .setIntakeDeliverMiddle()),
                                                                                                Commands.waitSeconds(
                                                                                                                1)),
                                                                                new SpeedCubeDeliverCmd(rc, 50, 0.8))),
                                                Map.entry(ManualDeliveryMethod.CubeHighNode,
                                                                new SequentialCommandGroup(
                                                                                new ParallelCommandGroup(
                                                                                                new InstantCommand(
                                                                                                                () -> rc.intakeSubsystem
                                                                                                                                .setIntakeDeliverUpper()),
                                                                                                Commands.waitSeconds(
                                                                                                                .5)),
                                                                                new SpeedCubeDeliverCmd(rc, 50, 1))),

                                                Map.entry(ManualDeliveryMethod.ClawLowNode,
                                                                new GripReleaseCmd(rc)),
                                                Map.entry(ManualDeliveryMethod.ClawMiddleNode,
                                                                new PositionLowPegCmd(rc)),
                                                Map.entry(ManualDeliveryMethod.ClawHighNode,
                                                                new SequentialCommandGroup(new PositionHighPegCmd(rc),
                                                                                new PositionHighPegSafetyCmd(rc)))),
                                                method));

        }
}
