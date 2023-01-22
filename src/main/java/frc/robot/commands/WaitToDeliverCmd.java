// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Add your docs here. */
public class WaitToDeliverCmd extends CommandBase{
    private double seconds;
    private double delay;
    public WaitToDeliverCmd(double delay){
        this.delay=delay;
    }
    @Override
    public void initialize(){
        seconds=delay;
    }
    @Override
    public void execute() {
        seconds-=0.2;
        
    }

    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("WaitToDeliverDone", seconds<=0);
        return seconds<=0;
    }
}
