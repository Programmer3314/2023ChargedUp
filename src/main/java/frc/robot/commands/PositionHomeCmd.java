// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class PositionHomeCmd extends CommandBase {
    private final RobotContainer rc;

    public PositionHomeCmd(RobotContainer rc) {
        this.rc = rc;
        addRequirements(rc.intakeSubsystem);
    }

    @Override
    public void initialize() {
        // if (rc.intakeSubsystem.robotHomed()) {
        // rc.intakeSubsystem.alreadyHomed();
        // } else {
        rc.intakeSubsystem.resetHomeStateMachine();
        rc.intakeSubsystem.setIntakeTravel();
        // rc.intakeSubsystem.setHomeNoExtend();
        // }

    }

    @Override
    public void execute() {
        if (rc.intakeSubsystem.robotHomed() && (rc.intakeSubsystem.getArmExtend() < .5)) {
            rc.intakeSubsystem.setHomeNoExtend();
        }

        SmartDashboard.putBoolean("setHome", rc.intakeSubsystem.robotHomed());
        SmartDashboard.putBoolean("Finished with PositionHome", Math.abs(rc.intakeSubsystem.getArmExtend() - 0) < .01
                && rc.intakeSubsystem.getArmRotate() < .03
                && rc.intakeSubsystem.robotHomed());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(rc.intakeSubsystem.getArmExtend() - 0) < .01
                && rc.intakeSubsystem.getArmRotate() < .03
                && rc.intakeSubsystem.robotHomed();
    }

    @Override
    public void end(boolean interrupted) {
        // rc.intakeSubsystem.homeRobot();
        // rc.intakeSubsystem.iterateExtendCounter();
        rc.intakeSubsystem.stopArm();
    }

}
