// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.utility.MMStateMachine;

enum HomeStates {
    Start, Safety, Speed, MoveToHome, Home, Normal
}

/** Add your docs here. */
public class HomeExtentionStateMachine extends MMStateMachine<HomeStates> {
    private RobotContainer rC;
    boolean lastHome = false;
    double startUpPosition;
    boolean hasHomed = false;

    public HomeExtentionStateMachine(RobotContainer rC) {
        super(HomeStates.Safety);
        this.rC = rC;
    }

    // TODO: fix problem with arm slamming into the retracted hard stop
    // There are times when we may not be extending past the CloseToHomeSensor,
    // such as Loading. When we retract we have homed so we go to speed, 
    // but if we're to close to home we never hit the CloseToHome sensor so
    // we go full force into the hard stop. 
    // Please add a condition for: hasHomed and distance is near home enough
    // that we should move slowly. You may want to add this to Safety and Speed.
    @Override
    public void CalcNextState() {
        switch (currentState) {
            case Start:
                nextState = HomeStates.Safety;
                break;
            case Safety:
                if (hasHomed) {
                    nextState = HomeStates.Speed;
                }
                if (rC.intakeSubsystem.getArmExtend() < startUpPosition - 0.1) {
                    nextState = HomeStates.Speed;
                }
                if (rC.intakeSubsystem.getCloseToHomeSensor()) {
                    nextState = HomeStates.MoveToHome;
                }
                if (rC.intakeSubsystem.getHomeSensor()) {
                    nextState = HomeStates.Home;
                }
                break;
            case Speed:
                if (rC.intakeSubsystem.getCloseToHomeSensor()) {
                    nextState = HomeStates.MoveToHome;
                }
                if (rC.intakeSubsystem.getHomeSensor()) {
                    nextState = HomeStates.Home;
                }
                break;
            case MoveToHome:
                if (rC.intakeSubsystem.getHomeSensor()) {
                    nextState = HomeStates.Home;
                }
                break;
            case Home:
                nextState = HomeStates.Normal;
                break;
            default:
                break;
        }
    }

    @Override
    public void doTransition() {

        // if (isTransitionFrom(HomeStates.Home)) {
        // rC.intakeSubsystem.stopArmExtend();
        // }
        // if (isTransitionFrom(HomeStates.Speed)) {
        // rC.intakeSubsystem.stopArmExtend();
        // }
        // if(isTransitionFrom(HomeStates.Safety)){
        // rC.intakeSubsystem.stopArmExtend();
        // }
        if (isTransitionTo(HomeStates.Safety)) {
            rC.intakeSubsystem.setHomeSlow();
            startUpPosition = rC.intakeSubsystem.getArmExtend();
        }
        if (isTransitionTo(HomeStates.Home)) {
            rC.intakeSubsystem.stopArmExtend();
        }
        if (isTransitionTo(HomeStates.Speed)) {
            rC.intakeSubsystem.setHomeFast();
        }
        if (isTransitionTo(HomeStates.MoveToHome)) {
            rC.intakeSubsystem.setHomeSlow();
        }
        if (isTransitionTo(HomeStates.Normal)) {
            rC.intakeSubsystem.stopArmExtend();
            rC.intakeSubsystem.resetExtendMotorEncoder();
            rC.intakeSubsystem.homeRobot();
            hasHomed = true;

        }

    }

    @Override
    public void doCurrentState() {
        if (currentState == HomeStates.Normal) {
            if (rC.intakeSubsystem.getHomeSensor() && !lastHome) {
                rC.intakeSubsystem.resetExtendMotorEncoder();
            }
            lastHome = rC.intakeSubsystem.getHomeSensor();
        }

    }

    public void resetState() {
        currentState = HomeStates.Start;
    }

    public HomeStates getState() {
        return currentState;
    }

    public boolean isHomed() {
        return currentState == HomeStates.Normal;
    }
}
