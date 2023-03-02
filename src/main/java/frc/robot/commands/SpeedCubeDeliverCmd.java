// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class SpeedCubeDeliverCmd extends CommandBase {
    private RobotContainer rc;
    private double delay;
    private double speed;
    private double currentCycle;

    public SpeedCubeDeliverCmd(RobotContainer rc, double delay, double speed) {
        this.rc = rc;
        this.delay = delay;
        this.speed = speed;

    }

    @Override
    public void initialize() {
        currentCycle = 0;
        rc.intakeSubsystem.runOutTakeSpeed(speed);
    }

    @Override
    public void execute() {
        currentCycle++;
    }

    @Override
    public boolean isFinished() {
        return currentCycle >= delay;
    }

    @Override
    public void end(boolean interrupted) {
        rc.intakeSubsystem.stopIntake();
    }
}
