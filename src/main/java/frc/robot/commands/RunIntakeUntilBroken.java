// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MMIntakeSubsystem;

/** Add your docs here. */
public class RunIntakeUntilBroken extends CommandBase{
    MMIntakeSubsystem intakeSubsystem;
    public RunIntakeUntilBroken(MMIntakeSubsystem intakeSubsystem){
        this.intakeSubsystem=intakeSubsystem;
    }
    @Override
    public void initialize() {
        intakeSubsystem.runIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
       return intakeSubsystem.getBeamBreak();
    }
}
