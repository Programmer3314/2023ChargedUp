// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class DeliverCubeHighCmd extends SequentialCommandGroup {
    public DeliverCubeHighCmd(MMSwerveSubsystem swerveSubsystem, double maxRotationSpeed,
            MMNavigationSubsystem navigationSubsystem, Supplier<Boolean> isRedAlliance) {
        addCommands(
                Commands.race(
                        new TargetTagCmd(swerveSubsystem, 2, navigationSubsystem),
                        new WaitToDeliverCmd(2)),
                new DriveToBumperCmd(navigationSubsystem, swerveSubsystem, .5),
                new TranslateAbsoluteCmd(swerveSubsystem,
                        () -> new Pose2d(
                                Constants.targetPositions.fieldXCoordinate
                                        * (isRedAlliance.get() ? 1
                                                : -1),
                                navigationSubsystem.getPose().getY(),
                                new Rotation2d(isRedAlliance.get() ? 0
                                        : Math.PI)),
                        1, navigationSubsystem));

    }

}
