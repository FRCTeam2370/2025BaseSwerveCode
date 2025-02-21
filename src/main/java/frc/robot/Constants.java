// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Lib.Utils.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public class Constants {
    public static class SwerveConstants {
        public static final double DrivekP = 0.2;
        public static final double DrivekI = 0;
        public static final double DrivekD = 0;

        public static final double TurnkP = 4;
        public static final double TurnkI = 0;
        public static final double TurnkD = 0;

        public static final double driveRamp = 0.2;

        public static final double maxSpeed = 5.21208;//meters per second
        public static final double maxAngularVelocity = 1;

        public static final double wheelRadius = 2;
        public static final double wheelCircumference = (2 * Math.PI) * wheelRadius;
        public static final double wheelCircumferenceMeters = wheelCircumference * 0.0254;

        public static final int pigeonID = 1;

        public static final double trackWidth = 22.5;
        public static final double wheelBase = 20.5;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d[]{
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            }
        );
    }

    //MODULE 1
    public static class FRConstants {
        public static final int driveMotorID = 21;
        public static final int turnMotorID = 22;
        public static final int CANCoderID = 23;

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(165.91375);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants FRConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 0
    public static class FLConstants {
        public static final int driveMotorID = 11;
        public static final int turnMotorID = 12;
        public static final int CANCoderID = 13;

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(353.49609);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants FLConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 3
    public static class BRConstants {
        public static final int driveMotorID = 31;
        public static final int turnMotorID = 32;
        public static final int CANCoderID = 33;

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(218.84765);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants BRConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 2
    public static class BLConstants {
        public static final int driveMotorID = 41;
        public static final int turnMotorID = 42;
        public static final int CANCoderID = 43;

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.Clockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(209.09791);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants BLConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }
}
