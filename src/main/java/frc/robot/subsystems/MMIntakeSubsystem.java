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
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class MMIntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    // private final AnalogInput intakeUltraSonic;
    private final DigitalInput intakeBeamBreak;
    private final DigitalInput intakeRightBreak;
    private final DigitalInput intakeLeftBreak;
    private final PneumaticsControlModule pneumaticsControlModule;
    private final DoubleSolenoid pickPosition;
    private final DoubleSolenoid shootPosition;

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
    private int resetExtendCounter = 0;
    private TalonFX outTakeMotor;

    public MMIntakeSubsystem(RobotContainer rc) {
        this.rc = rc;
        homeStateMachine = new HomeExtentionStateMachine(rc);
        intakeMotor = new TalonFX(Constants.DeliveryMotor.IntakeMotor.intakeMotorCanId);
        outTakeMotor = new TalonFX(Constants.DeliveryMotor.OutTakeMotor.outTakeMotorCanId);
        // intakeUltraSonic = new
        // AnalogInput(Constants.RoboRio.Analog.IntakeSensors.ultraSonicSensor);
        intakeBeamBreak = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.beamBreakSensor);
        intakeLeftBreak = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.breakBeamLeft);
        intakeRightBreak = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.breakBeamRight);

        pneumaticsControlModule = new PneumaticsControlModule(Constants.Pneumatic.pneumaticHubModule);
        pickPosition = pneumaticsControlModule.makeDoubleSolenoid( // large pneumatics that can set the intake to
                                                                   // floor or travel for the claw thing
                Constants.Pneumatic.LowerIntake.forwardChannel,
                Constants.Pneumatic.LowerIntake.reverseChannel);

        shootPosition = pneumaticsControlModule.makeDoubleSolenoid(
                Constants.Pneumatic.UpperIntake.forwardChannel, Constants.Pneumatic.UpperIntake.reverseChannel);

        armExtend = new TalonFX(Constants.Arm.Extend.CanID.Id);
        armRotate = new TalonFX(Constants.Arm.Rotation.CanID.Id);
        armHome = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.armHome);
        armCloseToHome = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.armCloseToHome);
        armFarFromHome = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.armFarFromHome);
        armAbsoluteRotation = new CANCoder(Constants.Arm.Encoder.Id);
        armAbsoluteRotation.configSensorDirection(true);
        armAbsoluteRotation.configMagnetOffset(-102);
        armAbsoluteRotation.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        armExtend.configFactoryDefault(Constants.Robot.canBusTimeoutMs);

        gripper = pneumaticsControlModule.makeDoubleSolenoid(Constants.Arm.Gripper.forwardChannel,
                Constants.Arm.Gripper.reverseChannel);
        spideySense = new DigitalInput(Constants.RoboRio.Dio.IntakeSensors.spideySense);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        armExtend.getAllConfigs(configs, Constants.Robot.canBusTimeoutMs);
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        armExtend.configAllSettings(configs, Constants.Robot.canBusTimeoutMs);
        armExtend.setNeutralMode(NeutralMode.Brake);
        armExtend.setInverted(true);
        armExtend.configPeakOutputForward(.5, Constants.Robot.canBusTimeoutMs);
        armExtend.configPeakOutputReverse(-.5, Constants.Robot.canBusTimeoutMs);
        armExtend.configAllowableClosedloopError(750, Constants.Robot.canBusTimeoutMs);
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
        armRotate.configPeakOutputForward(.5, Constants.Robot.canBusTimeoutMs);
        armRotate.configPeakOutputReverse(-.5, Constants.Robot.canBusTimeoutMs);
        armRotate.configAllowableClosedloopError(750, Constants.Robot.canBusTimeoutMs);

        armRotate.config_kF(0, Constants.Arm.Rotation.PIDValue.FF, Constants.Robot.canBusTimeoutMs);
        armRotate.config_kP(0, Constants.Arm.Rotation.PIDValue.P, Constants.Robot.canBusTimeoutMs);
        armRotate.config_kI(0, Constants.Arm.Rotation.PIDValue.I, Constants.Robot.canBusTimeoutMs);
        armRotate.config_kD(0, Constants.Arm.Rotation.PIDValue.D, Constants.Robot.canBusTimeoutMs);
        armRotate.config_IntegralZone(0, Constants.Arm.Rotation.PIDValue.Iz,
                Constants.Robot.canBusTimeoutMs);
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                enableCompressor();
                resetArmEncoder();
                // SmartDashboard.putString("Reset Running: ", "Yes");
            } catch (Exception e) {
                // SmartDashboard.putString("Reset Running: ", "Error");
            }
        }).start();

    }

    @Override
    public void periodic() {
        updateHomeStateMachine();

        SmartDashboard.putBoolean("Intake Beam Break: ", intakeBeamBreak.get());
        // SmartDashboard.putNumber("Intake UltraSonic: ",
        // intakeUltraSonic.getVoltage());
        SmartDashboard.putBoolean("Main Intake Beam", intakeBeamBreak.get());
        SmartDashboard.putBoolean("Left Beam Break", intakeLeftBreak.get());
        SmartDashboard.putBoolean("Right Beam Break", intakeRightBreak.get());
        SmartDashboard.putBoolean("Close to Home", getCloseToHomeSensor());
        SmartDashboard.putBoolean("Robot Homed Boolean", robotHomed);
        SmartDashboard.putBoolean("Arm Homed Sensor", getHomeSensor());
        SmartDashboard.putBoolean("Far From Home", getFarFromHomeSensor());
        SmartDashboard.putNumber("CanCoder Rotation DEG", Math.toDegrees(getCancoders()));
        SmartDashboard.putNumber("Motor Rotate Position", armRotate.getSelectedSensorPosition());
        SmartDashboard.putNumber("Extend Encoder", getArmExtend());
        SmartDashboard.putNumber("Arm Rotation", getArmRotate());
        SmartDashboard.putNumber("raw Rotation CanCoder", -armAbsoluteRotation.getAbsolutePosition());
        SmartDashboard.putNumber("Extend Counter", resetExtendCounter);
        SmartDashboard.putString("Current State", getCurrentState());

    }

    public void stopIntake() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
        outTakeMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void iterateExtendCounter() {
        resetExtendCounter++;
    }

    public boolean robotHomed() {
        return robotHomed;
    }

    public void homeRobot() {
        robotHomed = true;
    }

    public void runIntake() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, -.2);
    }

    public void runOutTakeSpeed(double speed) {
        intakeMotor.set(TalonFXControlMode.PercentOutput, speed);
        outTakeMotor.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void runOutTakeSlow() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, .2);
        outTakeMotor.set(TalonFXControlMode.PercentOutput, .2);
    }

    public void runOutTakeHigh() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, 1);
        outTakeMotor.set(TalonFXControlMode.PercentOutput, 1);
    }

    public void runOutTakeMiddle() {
        intakeMotor.set(TalonFXControlMode.PercentOutput, .85);
        outTakeMotor.set(TalonFXControlMode.PercentOutput, .85);
    }

    public boolean getBeamBreak() {
        return !intakeBeamBreak.get();
    }

    // public double getUltraSonicVoltage() {
    // return intakeUltraSonic.getVoltage();
    // }

    public void setArmRotationVoltage() {
        armRotate.set(TalonFXControlMode.PercentOutput, .1);
    }

    public void stopArmRotation() {
        armRotate.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void setArmRotationNeg() {
        armRotate.set(TalonFXControlMode.PercentOutput, -.1);
    }

    public void setIntakeFloor() {
        pickPosition.set(Value.kForward);
        shootPosition.set(Value.kReverse);
    }

    public void setIntakeTravel() {
        pickPosition.set(Value.kReverse);
        shootPosition.set(Value.kReverse);
    }

    public void setIntakeDeliverUpper() {
        shootPosition.set(Value.kForward);
        pickPosition.set(Value.kReverse);
    }

    public void setIntakeDeliverMiddle() {
        shootPosition.set(Value.kForward);
        pickPosition.set(Value.kReverse);
    }

    public void setArmExtend(double armPositionInMeters) {
        if (armPositionInMeters < 0) {
            armPositionInMeters = 0;
        } else if (armPositionInMeters > Constants.Arm.Extend.PositionControl.maxExtendPositionInMeters) {
            armPositionInMeters = Constants.Arm.Extend.PositionControl.maxExtendPositionInMeters;
        }
        armExtend.set(TalonFXControlMode.Position,
                armPositionInMeters * Constants.Arm.ConversionFactors.extensionTicksPerMeter);
    }

    public double getArmExtend() {
        return (armExtend.getSelectedSensorPosition() / Constants.Arm.ConversionFactors.extensionTicksPerMeter);
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

    public void setHomeNoExtend() {
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

    public void setHighPegSafety() {
        setArmExtend(Constants.Arm.Extend.PositionControl.highPeg);
        setArmRotation(Constants.Arm.Rotation.PositionControl.highPegSafety);
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

    public void enableCompressor() {
        pneumaticsControlModule.enableCompressorDigital();
    }

    public double getCancoders() {
        return Math.toRadians(armAbsoluteRotation.getAbsolutePosition() - Constants.Arm.Encoder.OffsetDegrees);
    }

    public void resetArmEncoder() {
        armRotate.setSelectedSensorPosition(Constants.Arm.ConversionFactors.rotationTicksPerRadians * getCancoders());
    }

    public boolean getHomeSensor() {
        return !armHome.get();
    }

    public boolean getFarFromHomeSensor() {
        return !armFarFromHome.get();
    }

    public boolean getCloseToHomeSensor() {
        return !armCloseToHome.get();
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

    public String getCurrentState() {
        return homeStateMachine.getState().toString();
    }

    public void updateHomeStateMachine() {
        homeStateMachine.update();
    }

    public void resetExtendMotorEncoder() {
        armExtend.setSelectedSensorPosition(0);
    }

    public boolean extendIsHomed() {
        return homeStateMachine.isHomed();
    }

    public void setGrip() {
        gripper.set(Value.kForward);

    }

    public void setRetract() {
        gripper.set(Value.kReverse);

    }

    public boolean getSpideySense() {
        return spideySense.get();
    }

    public Object zeroHeading(Object object) {
        return null;
    }

    public void setLightCube(){

    }

    public void setLightCone(){
        
    }

}
