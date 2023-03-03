// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class OneBallAutoCmd extends SequentialCommandGroup {
    public OneBallAutoCmd(RobotContainer rc, Supplier<Pose2d> startingPosition, BooleanSupplier isOverChargingStation) {
        addCommands(
                new InstantCommand(() -> rc.setAutoPosition()),
                new InstantCommand(() -> rc.setChargingStation()),
                new InstantCommand(() -> rc.navigationSubsystem.zeroHeading((rc::getIsRedAlliance))),
                new InstantCommand(() -> rc.navigationSubsystem.resetOdometry(startingPosition.get())),
                new PositionHomeCmd(rc),
                new GripReleaseCmd(rc),
                new DriveToGridAlleyCmd(rc, () -> true),
                new DriveToCellCmd(rc,
                        () -> 4,
                        rc::getIsRedAlliance,
                        1, rc.navigationSubsystem),
                Commands.either(
                        new CompleteRampSequenceCmd(rc), new LockedInCmd(rc), isOverChargingStation));
    }
}
