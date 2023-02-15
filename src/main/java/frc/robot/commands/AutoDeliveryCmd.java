// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DeliveryMethod;
import frc.robot.RobotContainer;
import frc.robot.utility.MMField;

// TODO: Add DeliverCubeHigh

/** Add your docs here. */
public class AutoDeliveryCmd extends SequentialCommandGroup {
    public AutoDeliveryCmd(RobotContainer rc) {
        addCommands(
                new InstantCommand(() -> rc.navigationSubsystem.setFrontPipeline(0)),
                new DriveToGridAlleyCmd(rc, () -> true),
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
                                Map.entry(DeliveryMethod.DeliverMiddleCube,
                                        new DeliverCubeMiddleCmd(
                                                2,
                                                rc))),
                        rc::selectDeliveryMethod),
                new InstantCommand(rc::clearSelectedCell));
    }
}
