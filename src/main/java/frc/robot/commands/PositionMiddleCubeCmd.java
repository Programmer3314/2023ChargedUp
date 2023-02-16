// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class PositionMiddleCubeCmd extends CommandBase {
    RobotContainer rc;
    public PositionMiddleCubeCmd(RobotContainer rc){
        this.rc=rc;
        addRequirements(rc.intakeSubsystem);
    }
    @Override
    public void initialize(){
        rc.intakeSubsystem.setArmExtend(0);
        rc.intakeSubsystem.setArmRotation(0);
        rc.intakeSubsystem.setIntakeDeliverUpper();
    }
    @Override
    public void execute(){

    }
    @Override
    public boolean isFinished(){
        return Math.abs(rc.intakeSubsystem.getArmExtend()) <0.01 && Math.abs(rc.intakeSubsystem.getArmRotate()) <0.01;
    }
    @Override
    public void end(boolean interrupted){
        rc.intakeSubsystem.stopArm();
    }
}
