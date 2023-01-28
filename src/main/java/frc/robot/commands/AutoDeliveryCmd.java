// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.DeliveryMethod;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// TODO: consider passing in robotContainer instead of everything else? Maybe?
/** Add your docs here. */
public class AutoDeliveryCmd extends SequentialCommandGroup {
    public AutoDeliveryCmd(MMSwerveSubsystem swerveSubsystem, MMNavigationSubsystem navigationSubsystem,
            Supplier<Boolean> isRedAlliance, Supplier<Integer> gridCell,
            Supplier<Object> selectDeliveryMethod) {
        addCommands(
                new InstantCommand(() -> navigationSubsystem.setPipeline(0)),

                new TranslateAbsoluteCmd(swerveSubsystem,
                        () -> new Pose2d(
                                Constants.targetPositions.fieldXCoordinate
                                        * (isRedAlliance.get() ? 1
                                                : -1),
                                navigationSubsystem.getPose().getY(),
                                new Rotation2d(isRedAlliance.get() ? 0
                                        : Math.PI)),
                        1, navigationSubsystem),
                new DriveToCell(swerveSubsystem,
                        () -> gridCell.get(),
                        navigationSubsystem,
                        () -> isRedAlliance.get(),
                        1),
                Commands.select(
                        Map.ofEntries(
                                Map.entry(DeliveryMethod.DeliverCone,
                                        new DeliverConeCmd(
                                                navigationSubsystem,
                                                2,
                                                swerveSubsystem,
                                                isRedAlliance)),
                                Map.entry(DeliveryMethod.DeliverFloorCube,
                                        new DeliverCubeFloorCmd(
                                                swerveSubsystem,
                                                2,
                                                navigationSubsystem,
                                                isRedAlliance)),
                                Map.entry(DeliveryMethod.DeliverHighCube,
                                        new DeliverCubeHighCmd(
                                                swerveSubsystem,
                                                2,
                                                navigationSubsystem,
                                                isRedAlliance))),
                        selectDeliveryMethod));
    }
}
