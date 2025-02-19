// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Lib.Utils.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor, turnMotor;
    private TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();

    private CANcoder turnEncoder;
    private CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    private SwerveModuleConstants moduleConstants;

    private Rotation2d lastAngle;

    private PositionDutyCycle angleDutyCyle = new PositionDutyCycle(0);
    private VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);

    public int moduleNumber;

    private SwerveModuleState currentState;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;
        this.moduleNumber = moduleNumber;

        turnEncoder = new CANcoder(moduleConstants.CANCoderID);
        configEncoder();

        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        turnMotor = new TalonFX(moduleConstants.turnMotorID);
        configModule();

        lastAngle = getState().angle;
    }

    public void configModule(){  
        driveMotorConfig.MotorOutput.Inverted = moduleConstants.driveInverted;
        turnMotorConfig.MotorOutput.Inverted = moduleConstants.turnInverted;

        driveMotorConfig.Slot0.kP = Constants.SwerveConstants.DrivekP;
        driveMotorConfig.Slot0.kI = Constants.SwerveConstants.DrivekI;
        driveMotorConfig.Slot0.kD = Constants.SwerveConstants.DrivekD; 
        //driveMotorConfig.Slot0.kV = (1.51 / 12);

        turnMotorConfig.Slot0.kP = Constants.SwerveConstants.TurnkP;
        turnMotorConfig.Slot0.kI = Constants.SwerveConstants.TurnkI;
        turnMotorConfig.Slot0.kD = Constants.SwerveConstants.TurnkD;

        turnMotorConfig.Feedback.SensorToMechanismRatio = (150.0 / 7.0);

        turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.driveRamp;

        driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotor.getConfigurator().apply(turnMotorConfig);

        driveMotor.setPosition(0);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);// turn these back to brake after offsets
        turnMotor.setNeutralMode(NeutralModeValue.Brake);

        resetToAbsolute();
    }

    public void configEncoder() {
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderConfig.MagnetSensor.MagnetOffset = 0;
        encoderConfig.MagnetSensor.SensorDirection = moduleConstants.EncoderReversed;

        turnEncoder.getConfigurator().apply(encoderConfig);
    }

    public void setDesiredState(SwerveModuleState state, boolean isOpenLoop){
        state.optimize(getState().angle);
        setAngle(state);
        setSpeed(state, isOpenLoop);
        SmartDashboard.putNumber("Module angle", state.angle.getDegrees());
        System.out.println("state degrees" + state.angle.getDegrees());
        System.out.println("Module Speed" + state.speedMetersPerSecond);
        currentState = state;
    }

    public double getWheelMPS(){
        return krakenToMPS(driveMotor.getVelocity().getValueAsDouble());
    }

    public double getModuleMeters(){
        return krakenToMeters(driveMotor.getPosition().getValueAsDouble());
    }

    public Rotation2d getCANcoder() {
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().waitForUpdate(1).getValueAsDouble());
    }

    public StatusSignal<Angle> getCANcoderVal() {
        return turnEncoder.getAbsolutePosition();
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble());                                                                                            // Math.abs
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            double output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(output);
        }else if(isOpenLoop == false){
            double velocity =  MPSToKraken(desiredState.speedMetersPerSecond);
            driveMotor.setControl(velocityDutyCycle.withVelocity(velocity));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle;
        turnMotor.setControl(angleDutyCyle.withPosition(angle.getRotations()));
        lastAngle = angle;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(krakenToMPS(driveMotor.getVelocity().getValueAsDouble()), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(krakenToMeters(driveMotor.getPosition().getValueAsDouble()), getAngle());
    }

    public void resetToAbsolute() {
        double cancoderpos = getCANcoder().getRotations();
        double absolutePosition = cancoderpos - moduleConstants.CANCoderOffset.getRotations();
        turnMotor.setPosition(absolutePosition);
        turnMotor.setControl(angleDutyCyle.withPosition(getState().angle.getRotations()));
        System.out.println("abosolute Pos on Start up" + absolutePosition);
    }


    // CONVERSIONS
    private double krakenToRPM(double rps) {
        double motorRPM = rps * 60;// don't know what these numbers are it may be (600 / 2048) instead
        double mechRPM = motorRPM / 6.12;// gear ratio of l3 drives
        return mechRPM;
    }

    private double RPMToKraken(double RPM) {
        double motorRPM = RPM * 6.12;
        return motorRPM;
    }

    private double krakenToMPS(double krakenRPS) {
        // double wheelRPM = krakenToRPM(vel);
        // double wheelMPS = (wheelRPM * Constants.SwerveConstants.wheelCircumference) / 60;                                                           // know
        // return wheelMPS;
        double wheelrps = krakenRPS / 6.12;
        double wheelMPS = wheelrps * Constants.SwerveConstants.wheelCircumferenceMeters;
        return wheelMPS;
    }

    private double MPSToKraken(double mps){
        double wheelRPM = mps / Constants.SwerveConstants.wheelCircumferenceMeters * 60;
        return RPMToKraken(wheelRPM);
    }

    private double krakenToMeters(double rot) {
        return rot * (Constants.SwerveConstants.wheelCircumferenceMeters / 6.12);
    }

    private double metersToKraken(double meters) {
        return meters / (Constants.SwerveConstants.wheelCircumferenceMeters / 6.12);
    }
}