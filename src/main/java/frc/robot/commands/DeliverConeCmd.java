// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MMIntakeSubsystem;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class DeliverConeCmd extends SequentialCommandGroup {

    public DeliverConeCmd(double maxRotationSpeed,
            RobotContainer rc) {
        addCommands(
                Commands.race(
                        new TargetPegCmd(rc.swerveSubsystem, 2, rc.navigationSubsystem,
                                rc.intakeSubsystem::getBeamBreak),
                        new WaitToDeliverCmd(2)),
                new DriveToBumperCmd(rc, .5),
                new TranslateAbsoluteCmd(rc.swerveSubsystem,
                        () -> new Pose2d(
                                Constants.targetPositions.fieldXCoordinate
                                        * (rc.getIsRedAlliance() ? 1
                                                : -1),
                                rc.navigationSubsystem.getPose().getY(),
                                new Rotation2d(rc.getIsRedAlliance() ? 0
                                        : Math.PI)),
                        1, rc.navigationSubsystem));
    }
}
