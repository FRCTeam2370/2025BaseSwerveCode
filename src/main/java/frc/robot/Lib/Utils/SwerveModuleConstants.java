// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SwerveModuleConstants {
    public int driveMotorID, turnMotorID, CANCoderID;
    public InvertedValue driveInverted, turnInverted;
    public Rotation2d CANCoderOffset;
    public SensorDirectionValue EncoderReversed;

    public SwerveModuleConstants(int driveMotorID, int turnMotorID, int CANCoderID, InvertedValue driveInverted, InvertedValue turnInverted, Rotation2d CANCoderOffset, SensorDirectionValue EncoderReversed){
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.CANCoderID = CANCoderID;
        this.driveInverted = driveInverted;
        this.turnInverted = turnInverted;
        this.CANCoderOffset = CANCoderOffset;
        this.EncoderReversed = EncoderReversed;
    }
}
