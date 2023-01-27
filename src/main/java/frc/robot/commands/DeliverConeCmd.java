// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MMNavigationSubsystem;
import frc.robot.subsystems.MMSwerveSubsystem;

/** Add your docs here. */
public class DeliverConeCmd extends SequentialCommandGroup {

    public DeliverConeCmd(MMNavigationSubsystem navigationSubsystem, double maxRotationSpeed,
            MMSwerveSubsystem swerveSubsystem) {
        addCommands(
                Commands.race(
                        new TargetPegCmd(swerveSubsystem, 2, navigationSubsystem),
                        new WaitToDeliverCmd(30)),
                new DriveToPegCmd(navigationSubsystem, swerveSubsystem, .5));
    }
}
