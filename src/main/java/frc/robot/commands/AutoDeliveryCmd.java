// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.DeliveryMethod;
import frc.robot.RobotContainer;
import frc.robot.utility.MMField;

// TODO: consider passing in robotContainer instead of everything else? Maybe?
/** Add your docs here. */
public class AutoDeliveryCmd extends SequentialCommandGroup {
    public AutoDeliveryCmd(RobotContainer rc) {
        addCommands(
                new InstantCommand(() -> rc.navigationSubsystem.setFrontPipeline(0)),
                new TranslateAbsoluteCmd(rc,
                        () -> new Pose2d(
                                Constants.targetPositions.fieldXCoordinate
                                        * (rc.getIsRedAlliance() ? 1
                                                : -1),
                                rc.navigationSubsystem.getPose().getY(),
                                new Rotation2d(rc.getIsRedAlliance() ? 0
                                        : Math.PI)),
                        1),
                new DriveToCellCmd(rc,
                        rc::getGridCell,
                        rc::getIsRedAlliance,
                        1),

                Commands.select(
                        Map.ofEntries(
                                Map.entry(DeliveryMethod.DeliverCone,
                                        new DeliverConeCmd(
                                                2, rc)),
                                Map.entry(DeliveryMethod.DeliverFloorCube,
                                        new DeliverCubeFloorCmd(
                                                2,
                                                () -> MMField.isCellCone(
                                                        rc.getGridCell()),
                                                rc)),
                                Map.entry(DeliveryMethod.DeliverHighCube,
                                        new DeliverCubeHighCmd(
                                                2,
                                                rc))),
                        rc::selectDeliveryMethod),
                new InstantCommand(rc::clearSelectedCell));
    }
}
