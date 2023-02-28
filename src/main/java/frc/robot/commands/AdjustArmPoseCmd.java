// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AdjustSelection;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class AdjustArmPoseCmd extends SequentialCommandGroup {

    public AdjustArmPoseCmd(Supplier<Object> selection, RobotContainer rc) {
        addCommands(
                Commands.select(Map.ofEntries(
                        Map.entry(AdjustSelection.RotateIncrease,
                                new InstantCommand(() -> rc.intakeSubsystem
                                        .setArmRotation(rc.intakeSubsystem.getArmRotate() + .0494))),
                        Map.entry(AdjustSelection.RotateDecrease,
                                new InstantCommand(() -> rc.intakeSubsystem
                                        .setArmRotation(rc.intakeSubsystem.getArmRotate() - .0494))),
                        Map.entry(AdjustSelection.ExtendIncrease,
                                new InstantCommand(() -> rc.intakeSubsystem
                                        .setArmExtend(rc.intakeSubsystem.getArmExtend() + .0254))),
                        Map.entry(AdjustSelection.ExtendDecrease,
                                new InstantCommand(() -> rc.intakeSubsystem
                                        .setArmExtend(rc.intakeSubsystem.getArmExtend() + .0254)))),
                        selection));

    }
}
