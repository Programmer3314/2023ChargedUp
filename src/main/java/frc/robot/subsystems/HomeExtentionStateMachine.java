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

    public HomeExtentionStateMachine(RobotContainer rC) {
        super(HomeStates.Safety);
        this.rC = rC;
    }

    @Override
    public void CalcNextState() {
        // TODO Auto-generated method stub
        switch (currentState) {
            case Start:
                nextState = HomeStates.Safety;
                break;
            case Safety:
                if (rC.intakeSubsystem.getArmExtend() < -0.1) {
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
        //     rC.intakeSubsystem.stopArmExtend();
        // }
        // if (isTransitionFrom(HomeStates.Speed)) {
        //     rC.intakeSubsystem.stopArmExtend();
        // }
        // if(isTransitionFrom(HomeStates.Safety)){
        //     rC.intakeSubsystem.stopArmExtend();
        // }
        if (isTransitionTo(HomeStates.Safety)){
            rC.intakeSubsystem.setHomeSlow();
        }
        if(isTransitionTo(HomeStates.Home)){
            rC.intakeSubsystem.stopArmExtend();
        }
        if (isTransitionTo(HomeStates.Speed)) {
            rC.intakeSubsystem.setHomeFast();
        }
        if (isTransitionTo(HomeStates.MoveToHome)) {
            rC.intakeSubsystem.setHomeSlow();
        }
        if (isTransitionTo(HomeStates.Normal)){
            rC.intakeSubsystem.stopArmExtend();
            rC.intakeSubsystem.resetExtendMotorEncoder();
            rC.intakeSubsystem.homeRobot();
            
        }

    }

    @Override
    public void doCurrentState() {

    }

    public void resetState() {
        currentState = HomeStates.Start;
    }

    public boolean isHomed() {
        return currentState == HomeStates.Normal;
    }
}
