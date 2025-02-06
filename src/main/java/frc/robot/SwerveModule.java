// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Lib.Utils.CTREModuleState;
import frc.robot.Lib.Utils.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private TalonFX driveMotor, turnMotor;
    private TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();

    private CANcoder turnEncoder;
    private CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    private SwerveModuleConstants moduleConstants;

    private Rotation2d lastAngle = new Rotation2d(0);

    private PositionDutyCycle angleDutyCyle = new PositionDutyCycle(0);

    public int moduleNumber;

    public SwerveModule(int modueleNumber, SwerveModuleConstants moduleConstants){
        this.moduleConstants = moduleConstants;
        driveMotor = new TalonFX(moduleConstants.driveMotorID);
        turnMotor = new TalonFX(moduleConstants.turnMotorID);
        turnEncoder = new CANcoder(moduleConstants.CANCoderID);
        this.moduleNumber = modueleNumber;
        // configModule();
        // configEncoder();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState state){
        state = CTREModuleState.optimize(state, lastAngle);
        setAngle(state);
        setSpeed(state);
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().waitForUpdate(0.5).getValueAsDouble());
    }   

    public StatusSignal<Angle> getCANcoderVal(){
        return turnEncoder.getAbsolutePosition();
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(krakenToDegrees(turnMotor.getPosition().getValueAsDouble()));
    }

    private void setSpeed(SwerveModuleState desiredState){
        double output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
        driveMotor.set(output);
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle;

        turnMotor.setControl(angleDutyCyle.withPosition(degreesToKraken(angle.getDegrees())));
        lastAngle = angle;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(krakenToMPS(driveMotor.getVelocity().getValueAsDouble()), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(krakenToMeters(driveMotor.getPosition().getValueAsDouble()), getAngle());
    }

    public void resetToAbsolute(){
        //turnMotor.setPosition(0);
        double absolutePosition = degreesToKraken(getCANcoder().getDegrees() - moduleConstants.CANCoderOffset.getDegrees());
        turnMotor.setPosition(absolutePosition);
        turnMotor.setControl(angleDutyCyle.withPosition(0));
    }

    public void configModule(){        
        driveMotorConfig.MotorOutput.Inverted = moduleConstants.driveInverted;
        turnMotorConfig.MotorOutput.Inverted = moduleConstants.turnInverted;

        driveMotorConfig.Slot0.kP = Constants.SwerveConstants.DrivekP;
        driveMotorConfig.Slot0.kI = Constants.SwerveConstants.DrivekI;
        driveMotorConfig.Slot0.kD = Constants.SwerveConstants.DrivekD;

        turnMotorConfig.Slot0.kP = Constants.SwerveConstants.TurnkP;
        turnMotorConfig.Slot0.kI = Constants.SwerveConstants.TurnkI;
        turnMotorConfig.Slot0.kD = Constants.SwerveConstants.TurnkD;

        driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotor.getConfigurator().apply(turnMotorConfig);

        driveMotor.setPosition(0);  
        driveMotor.setNeutralMode(NeutralModeValue.Brake);//turn these back to brake after offsets
        turnMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void configEncoder(){
        turnEncoder.getConfigurator().apply(new CANcoderConfiguration());

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.MagnetOffset = 0;//moduleConstants.CANCoderOffset.getRotations();
        encoderConfig.MagnetSensor.SensorDirection = moduleConstants.EncoderReversed;
        
        turnEncoder.getConfigurator().apply(encoderConfig);
    }


    //CONVERSIONS
    private double degreesToKraken(double degrees){
        return degrees / 360 * (150/7);//degrees / (360 / (150/7))
    }

    private double krakenToDegrees(double rot){
        return rot / (150/7) * 360; //rot * (360 / (150/7))
    }

    private double krakenToRPM(double vel){
        double motorRPM = vel * 600;//don't know what these numbers are it may be (600 / 2048) instead
        double mechRPM  = motorRPM / 6.12;//gear ratio of l3 drives
        return mechRPM;
    }

    private double RPMToKraken(double RPM){
        double motorRPM = RPM * 6.12;
        return motorRPM;
    }

    private double krakenToMPS(double vel){
        double wheelRPM = krakenToRPM(vel);
        double wheelMPS = wheelRPM * Constants.SwerveConstants.wheelCircumference / 60;// it may not be / 60 I don't know
        return wheelMPS;
    }

    private double krakenToMeters(double rot){
        return rot * (Constants.SwerveConstants.wheelCircumference / 6.12);
    }

    private double metersToKraken(double meters){
        return meters / (Constants.SwerveConstants.wheelCircumference / 6.12);
    }
}
