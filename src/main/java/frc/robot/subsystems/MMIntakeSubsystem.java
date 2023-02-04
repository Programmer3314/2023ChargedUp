// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify andor share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Add your docs here. */
public class MMIntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final AnalogInput intakeUltraSonic;
    private final DigitalInput intakeBeamBreak;
    private final PneumaticHub pneumaticHub;
    private final DoubleSolenoid intakePosition;

    public MMIntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.DeliveryMotor.IntakeMotor.intakeMotorCanId);
        intakeUltraSonic = new AnalogInput(Constants.RoboRio.Analog.IntakeSensors.ultraSonicSensor);
        intakeBeamBreak = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.beamBreakSensor);
        pneumaticHub = new PneumaticHub(Constants.Pneumatic.pneumaticHubModule);
        intakePosition = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Pneumatic.intakeForwardChannel,
                Constants.Pneumatic.intakeReverseChannel);
    }

    public void runIntake() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, .5);
    }

    public void runOutTake() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, .5);
    }

    public boolean getBeamBreak() {
        return intakeBeamBreak.get();
    }

    public double getUltraSonicVoltage() {
        return intakeUltraSonic.getVoltage();
    }

    public void setIntakeFloor() {
        intakePosition.set(Value.kForward);
    }

    public void setIntakeTravel() {
        intakePosition.set(Value.kReverse);
    }

    public void enableCompressor(){
        pneumaticHub.enableCompressorDigital();
    }
}
