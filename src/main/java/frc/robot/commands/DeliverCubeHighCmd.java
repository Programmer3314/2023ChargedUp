// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DeliverCubeHighCmd extends SequentialCommandGroup {
        public DeliverCubeHighCmd(double maxRotationSpeed, RobotContainer rc) {
                addCommands(
                                new TranslateAbsoluteCmd(rc,
                                                () -> new Pose2d(
                                                                Constants.targetPositions.fieldXCoordinate
                                                                                * (rc.getIsRedAlliance() ? 1
                                                                                                : -1),
                                                                rc.navigationSubsystem.getPose().getY(),
                                                                new Rotation2d(rc.getIsRedAlliance() ? Math.PI
                                                                                : 0)),
                                                1),
                                Commands.race(
                                                new TargetTagCmd(rc, 2,
                                                                rc.intakeSubsystem::getBeamBreak),
                                                new WaitToDeliverCmd(2)),
                                new DriveToBumperCmd(rc, .5),
                                new TranslateAbsoluteCmd(rc,
                                                () -> new Pose2d(
                                                                Constants.targetPositions.fieldXCoordinate
                                                                                * (rc.getIsRedAlliance() ? 1
                                                                                                : -1),
                                                                rc.navigationSubsystem.getPose().getY(),
                                                                new Rotation2d(rc.getIsRedAlliance() ? 0
                                                                                : Math.PI)),
                                                1));

        }

}
