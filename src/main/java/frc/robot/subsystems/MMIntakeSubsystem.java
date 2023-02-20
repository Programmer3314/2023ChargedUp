// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify andor share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


/** Add your docs here. */
public class MMIntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    //private final AnalogInput intakeUltraSonic;
    private final DigitalInput intakeBeamBreak;
    private final DigitalInput intakeRightBreak;
    private final DigitalInput intakeLeftBreak;
    // private final PneumaticHub pneumaticHub;
    private final DoubleSolenoid intakePosition;
    private final DoubleSolenoid intakeUpperPosition;

    private final TalonFX armExtend;
    private final TalonFX armRotate;
    private final DigitalInput armHome;
    private final DigitalInput armCloseToHome;
    private final DigitalInput armFarFromHome;
    private final CANCoder armAbsoluteRotation;
    private HomeExtentionStateMachine homeStateMachine;
    private RobotContainer rc;
    private boolean robotHomed = false;
    private DoubleSolenoid gripper;
    private DigitalInput spideySense;

    public MMIntakeSubsystem(RobotContainer rc) {
        this.rc=rc;
        homeStateMachine=new HomeExtentionStateMachine(rc);
        intakeMotor = new TalonFX(Constants.DeliveryMotor.IntakeMotor.intakeMotorCanId);
       // intakeUltraSonic = new AnalogInput(Constants.RoboRio.Analog.IntakeSensors.ultraSonicSensor);
        intakeBeamBreak = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.beamBreakSensor);
        intakeLeftBreak = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.breakBeamLeft);
        intakeRightBreak = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.breakBeamRight);

        // pneumaticHub = new PneumaticHub(Constants.Pneumatic.pneumaticHubModule);
        intakePosition = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                Constants.Pneumatic.LowerIntake.forwardChannel,
                Constants.Pneumatic.LowerIntake.reverseChannel);

        intakeUpperPosition = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                Constants.Pneumatic.UpperIntake.forwardChannel, Constants.Pneumatic.UpperIntake.reverseChannel);

        armExtend = new TalonFX(Constants.Arm.Extend.CanID.Id);
        armRotate = new TalonFX(Constants.Arm.Rotation.CanID.Id);
        armHome = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.armHome);
        armCloseToHome = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.armCloseToHome);
        armFarFromHome = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.armFarFromHome);
        armAbsoluteRotation = new CANCoder(Constants.Arm.Encoder.Id);
        armExtend.configFactoryDefault(Constants.Robot.canBusTimeoutMs);

        gripper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Arm.Gripper.forwardChannel, Constants.Arm.Gripper.reverseChannel);
        spideySense = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.spideySense);



        TalonFXConfiguration configs = new TalonFXConfiguration();

        armExtend.getAllConfigs(configs, Constants.Robot.canBusTimeoutMs);
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        armExtend.configAllSettings(configs, Constants.Robot.canBusTimeoutMs);
        armExtend.setNeutralMode(NeutralMode.Brake);
        armExtend.setInverted(false);
        armExtend.config_kF(0, Constants.Arm.Extend.PIDValue.FF, Constants.Robot.canBusTimeoutMs);
        armExtend.config_kI(0, Constants.Arm.Extend.PIDValue.I, Constants.Robot.canBusTimeoutMs);
        armExtend.config_kP(0, Constants.Arm.Extend.PIDValue.P, Constants.Robot.canBusTimeoutMs);
        armExtend.config_kD(0, Constants.Arm.Extend.PIDValue.D, Constants.Robot.canBusTimeoutMs);
        armExtend.config_IntegralZone(0, Constants.Arm.Extend.PIDValue.Iz,
                Constants.Robot.canBusTimeoutMs);
        armRotate.configFactoryDefault(Constants.Robot.canBusTimeoutMs);

        // TalonFXConfiguration extendConfigs = new TalonFXConfiguration();

        armRotate.getAllConfigs(configs, Constants.Robot.canBusTimeoutMs);
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        armRotate.configAllSettings(configs, Constants.Robot.canBusTimeoutMs);
        armRotate.setNeutralMode(NeutralMode.Brake);
        armRotate.setInverted(false);
        armRotate.config_kF(0, Constants.Arm.Rotation.PIDValue.FF, Constants.Robot.canBusTimeoutMs);
        armRotate.config_kP(0, Constants.Arm.Rotation.PIDValue.P, Constants.Robot.canBusTimeoutMs);
        armRotate.config_kI(0, Constants.Arm.Rotation.PIDValue.I, Constants.Robot.canBusTimeoutMs);
        armRotate.config_kD(0, Constants.Arm.Rotation.PIDValue.D, Constants.Robot.canBusTimeoutMs);
        armRotate.config_IntegralZone(0, Constants.Arm.Rotation.PIDValue.Iz,
                Constants.Robot.canBusTimeoutMs);
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                // resetArmEncoder();
                // SmartDashboard.putString("Reset Running: ", "Yes");
            } catch (Exception e) {
                // SmartDashboard.putString("Reset Running: ", "Error");
            }
        }).start();

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Beam Break: ", intakeBeamBreak.get());
       // SmartDashboard.putNumber("Intake UltraSonic: ", intakeUltraSonic.getVoltage());

    }

    public void stopIntake() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
    public boolean robotHomed(){
        return robotHomed;
        
    }
    public void homeRobot(){
        robotHomed=true;
    }

    public void runIntake() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, -.2);
    }

    public void runOutTakeSlow() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, .2);
    }

    public void runOutTake() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, 1);
    }

    public boolean getBeamBreak() {
        return !intakeBeamBreak.get();
    }

    // public double getUltraSonicVoltage() {
    //     return intakeUltraSonic.getVoltage();
    // }

    public void setIntakeFloor() {
        intakePosition.set(Value.kForward);
        intakeUpperPosition.set(Value.kForward);
    }

    public void setIntakeTravel() {
        intakePosition.set(Value.kReverse);
        intakeUpperPosition.set(Value.kForward);
    }

    public void setIntakeDeliverUpper() {
        intakeUpperPosition.set(Value.kReverse);
        intakePosition.set(Value.kReverse);

    }

    public void setIntakeDeliverLower() {
        intakeUpperPosition.set(Value.kReverse);
        intakePosition.set(Value.kReverse);
    }

    public void setArmExtend(double armPositionInMeters) {
        armExtend.set(TalonFXControlMode.Position,
                armPositionInMeters * Constants.Arm.ConversionFactors.extensionTicksPerMeter);
    }

    public double getArmExtend() {
        return (armExtend.getSelectedSensorPosition() / Constants.Arm.ConversionFactors.rotationTicksPerRadians);

    }

    public void setArmRotation(double radians) {
        armRotate.set(TalonFXControlMode.Position, radians * Constants.Arm.ConversionFactors.rotationTicksPerRadians);
    }

    public double getArmRotate() {
        return (armRotate.getSelectedSensorPosition() / Constants.Arm.ConversionFactors.rotationTicksPerRadians);
    }

    public void setHome() {
        setArmExtend(0);
        setArmRotation(0);
        setIntakeTravel();

    }
    public void setHomeNoExtend(){
        setArmRotation(0);
        setIntakeTravel();
    }

    public void setLoading() {
        setArmExtend(Constants.Arm.Extend.PositionControl.loading);
        setArmRotation(Constants.Arm.Rotation.PositionControl.loading);
        setIntakeTravel();
    }

    public void setHighPeg() {
        setArmExtend(Constants.Arm.Extend.PositionControl.highPeg);
        setArmRotation(Constants.Arm.Rotation.PositionControl.highPeg);
        setIntakeTravel();

    }

    public void setLowPeg() {
        setArmExtend(Constants.Arm.Extend.PositionControl.lowPeg);
        setArmRotation(Constants.Arm.Rotation.PositionControl.lowPeg);
        setIntakeTravel();

    }

    public void setGround() {
        setArmExtend(Constants.Arm.Extend.PositionControl.ground);
        setArmRotation(Constants.Arm.Rotation.PositionControl.ground);
        setIntakeTravel();

    }

    public void stopArm() {
        armExtend.set(ControlMode.PercentOutput, 0);
        armRotate.set(ControlMode.PercentOutput, 0);

    }

    // public void enableCompressor() {
    // pneumaticHub.enableCompressorDigital();
    // }
    public double getCancoders() {
        return Math.toRadians(armAbsoluteRotation.getAbsolutePosition() - Constants.Arm.Encoder.Offset);

    }

    public void resetArmEncoder() {
        armRotate.setSelectedSensorPosition(Constants.Arm.ConversionFactors.rotationTicksPerRadians * getCancoders());
    }

    public boolean getHomeSensor() {
        return armHome.get();
    }

    public boolean getFarFromHomeSensor() {
        return armFarFromHome.get();
    }

    public boolean getCloseToHomeSensor() {
        return armCloseToHome.get();
    }

    public void setHomeSlow() {
        armExtend.set(ControlMode.PercentOutput, Constants.Arm.Extend.SpeedControl.safety);
    }

    public void setHomeFast() {
        armExtend.set(ControlMode.PercentOutput, Constants.Arm.Extend.SpeedControl.speed);
    }

    public void stopArmExtend() {
        armExtend.set(ControlMode.PercentOutput, 0);
    }

    public void resetHomeStateMachine() {
        homeStateMachine.resetState();
    }

    public void updateHomeStateMachine(){
        homeStateMachine.update();
    }

    public void resetExtendMotorEncoder(){
        armExtend.setSelectedSensorPosition(0);
    }

    public boolean extendIsHomed(){
        return homeStateMachine.isHomed();
    }
    public void setGrip(){
        gripper.set(Value.kForward);

    }
    public void setRetract(){
        gripper.set(Value.kReverse);

    }
    public boolean getSpideySense(){
        return spideySense.get();
    }
    
}
