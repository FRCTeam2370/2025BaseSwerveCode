// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.spi.CurrencyNameProvider;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.ImmutableLinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Lib.Utils.CTREModuleState;
import frc.robot.Lib.Utils.Optimize;
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

    public int moduleNumber;

    public SwerveModule(int modueleNumber, SwerveModuleConstants moduleConstants){
        this.moduleConstants = moduleConstants;
        this.moduleNumber = modueleNumber;

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

        turnMotorConfig.Slot0.kP = Constants.SwerveConstants.TurnkP;
        turnMotorConfig.Slot0.kI = Constants.SwerveConstants.TurnkI;
        turnMotorConfig.Slot0.kD = Constants.SwerveConstants.TurnkD;

        //driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.3;

        driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotor.getConfigurator().apply(turnMotorConfig);

        driveMotor.setPosition(0);  
        driveMotor.setNeutralMode(NeutralModeValue.Brake);//turn these back to brake after offsets
        turnMotor.setNeutralMode(NeutralModeValue.Brake);

        //resetToAbsolute();
    }

    public void configEncoder(){
        //turnEncoder.getConfigurator().apply(new CANcoderConfiguration());

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderConfig.MagnetSensor.MagnetOffset = 0; //moduleConstants.CANCoderOffset.getRotations();
        encoderConfig.MagnetSensor.SensorDirection = moduleConstants.EncoderReversed;
        
        turnEncoder.getConfigurator().apply(encoderConfig);
    }

    public void setDesiredState(SwerveModuleState state){
        //state = optimize(state, getState().angle);
        //state = CTREModuleState.optimize(state, getState().angle);
        //state = optimizeState(state, lastAngle);
        //state = Optimize.SwerveOptimizeAngle(state, lastAngle);
        setAngle(state);
        setSpeed(state);
        SmartDashboard.putNumber("Module angle", state.angle.getDegrees());
        System.out.println("state.angle.getDegrees()" + state.angle.getDegrees());
        System.out.println("state degrees" + state.angle.getDegrees());
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(turnEncoder.getAbsolutePosition().waitForUpdate(1).getValueAsDouble());
    }

    public StatusSignal<Angle> getCANcoderVal(){
        return turnEncoder.getAbsolutePosition();
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(krakenToDegrees(turnMotor.getPosition().getValueAsDouble()));//changed to Math.abs
    }

    private void setSpeed(SwerveModuleState desiredState){
        double output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
        driveMotor.set(output);
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle;
        Rotation2d oldAngle = getAngle();
        angle = optimizeTurn2(oldAngle, angle);
        turnMotor.setControl(angleDutyCyle.withPosition(degreesToKraken(angle.getDegrees())));
        lastAngle = angle;
    }

    private void setAngleInit(Rotation2d angle){
        Rotation2d oldAngle = getAngle();
        angle = optimizeTurn(oldAngle, angle);
        turnMotor.setControl(angleDutyCyle.withPosition(degreesToKraken(angle.getDegrees())));
        lastAngle = angle;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(krakenToMPS(driveMotor.getVelocity().getValueAsDouble()), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(krakenToMeters(driveMotor.getPosition().getValueAsDouble()), getAngle());
    }

    //You  NEED this method in order to pass a positive value into the CTRESwerveModuleState.optimize() function so the modules don't spin on startup
    public double makePosDegrees(double CANcoderPos, double offset) {
        // if(offset > CANcoderPos){
        //     offset -= 360;
        // }

        // return CANcoderPos - offset;

        if(offset > 180 && CANcoderPos <= 180){
            return (360 - offset) + CANcoderPos; 
        }else if(offset > 180 && offset < CANcoderPos){
            return CANcoderPos - offset;
        }else if(offset < 180 && offset > CANcoderPos){
            return CANcoderPos - offset + 360;
        }else{
            return CANcoderPos - offset;
        }
    }

    public static double makePositiveDegrees(double anAngle) {
        double degrees = anAngle;
        degrees = degrees % 360;
        if (degrees < 0.0){
            degrees = degrees + 360;
        }
        return degrees;

    }

    public static double makePositiveDegrees(Rotation2d anAngle){
        return makePositiveDegrees(anAngle.getDegrees());
    }

    public Rotation2d optimizeTurn(Rotation2d oldAngle, Rotation2d newAngle){
        double steerAngle = newAngle.getDegrees();//makePositiveDegrees(newAngle);
        steerAngle %= (360);
        if (steerAngle < 0.0) {
            steerAngle += 360;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360) instead of [0, 360)
        if (difference >= 360) {
            steerAngle -= 360;//360
        } else if (difference < -360) {
            steerAngle += 360;//360
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference >90 || difference < -90) {//90, -90
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(steerAngle);//makePositiveDegrees(steerAngle)
    }

    public Rotation2d optimizeTurn2(Rotation2d oldAngle, Rotation2d newAngle){
        double steerAngle = newAngle.getDegrees();//makePositiveDegrees(newAngle);
        steerAngle %= (180);
        if (steerAngle < 0.0) {
            steerAngle += 180;
        }

        double difference = steerAngle - oldAngle.getDegrees();
        // Change the target angle so the difference is in the range [-360, 360) instead of [0, 360)
        if (difference >= 180) {
            steerAngle -= 180;//360
        } else if (difference < -180) {
            steerAngle += 180;//360
        }
        difference = steerAngle - oldAngle.getDegrees(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference >90 || difference < -90) {//90, -90
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += 180;
        }

        return Rotation2d.fromDegrees(steerAngle);//makePositiveDegrees(steerAngle)
    }

    public void resetToAbsolute(){
        //turnMotor.setPosition(0);
        double cancoderpos = getCANcoder().getDegrees();
        double absolutePosition = degreesToKraken(makePositiveDegrees(cancoderpos - moduleConstants.CANCoderOffset.getDegrees()));
        turnMotor.setPosition(absolutePosition);
        turnMotor.setControl(angleDutyCyle.withPosition(degreesToKraken(getState().angle.getDegrees())));
        System.out.println("abosolute Pos on Start up" + absolutePosition);
    }

    private static SwerveModuleState optimizeState(SwerveModuleState state, Rotation2d currentAngle){
        double angle = state.angle.getDegrees() % 360;
        double lastAngle = currentAngle.getDegrees() % 360;
        double dir = angle - lastAngle;

        return new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(dir));
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.rotateBy(currentAngle.unaryMinus());    // todo check math
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
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
