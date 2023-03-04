// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class OneConeLeaveCommunityAutoCmd extends SequentialCommandGroup {
  public OneConeLeaveCommunityAutoCmd(RobotContainer rc, Supplier<Pose2d> startingPosition) {
    addCommands(
        new InstantCommand(() -> rc.setAutoPosition()),
        new InstantCommand(() -> rc.setChargingStation()),
        new InstantCommand(() -> rc.navigationSubsystem.zeroHeading((rc::getIsRedAlliance))),
        new InstantCommand(() -> rc.navigationSubsystem.resetOdometry(startingPosition.get())),
        new PositionHomeCmd(rc),
        Commands.waitSeconds(0.25),
        new PositionLowPegCmd(rc),
        new GripReleaseCmd(rc),
        new PositionHomeCmd(rc),
        new ParallelCommandGroup(
            new FullIntakeCmd(rc),
            new TranslateAbsoluteCmd(rc,
                () -> rc.pickUpOutsidePosition(), .2)));

  }

}
