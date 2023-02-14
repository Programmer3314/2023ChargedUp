// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

// TODO: Do Normal stuff if isHomed();
// otherwise do state machine 
// Add conditions to each method. 


/** Add your docs here. */
public class IntakeHome extends CommandBase {
    private final RobotContainer rc;

    public IntakeHome(RobotContainer rc) {
        this.rc = rc;
        addRequirements(rc.intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (rc.intakeSubsystem.robotHomed()){
            rc.intakeSubsystem.setHome();

        }else{
            rc.intakeSubsystem.resetHomeStateMachine();
            rc.intakeSubsystem.setHomeNoExtend();
        }

    }

    @Override
    public void execute() {
        rc.intakeSubsystem.updateHomeStateMachine();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rc.intakeSubsystem.getArmExtend() - 0) < .01
                && Math.abs(rc.intakeSubsystem.getArmRotate() - 0) < .01
                && rc.intakeSubsystem.robotHomed();
    }

    @Override
    public void end(boolean interrupted) {
        rc.intakeSubsystem.resetExtendMotorEncoder();
        rc.intakeSubsystem.stopArm();
    }

}
