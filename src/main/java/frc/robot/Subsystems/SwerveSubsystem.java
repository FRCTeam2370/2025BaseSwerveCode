// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.io.IOException;
import java.net.ContentHandler;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  public static Pigeon2 gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
  public SwerveModule[] mSwerveModules;
  public SwerveDriveOdometry odometry;
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    mSwerveModules = new SwerveModule[] {
      new SwerveModule(0, Constants.FLConstants.FLConstants),
      new SwerveModule(1, Constants.FRConstants.FRConstants),
      new SwerveModule(2, Constants.BLConstants.BLConstants),
      new SwerveModule(3, Constants.BRConstants.BRConstants)
    };

    odometry = new SwerveDriveOdometry(Constants.SwerveConstants.kinematics, getYaw(), getModulePositions());

    configurePathPlanner();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Mod 0 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[0].getCANcoder().getDegrees()).getDegrees());
    SmartDashboard.putNumber("Mod 1 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[1].getCANcoder().getDegrees()).getDegrees());
    SmartDashboard.putNumber("Mod 2 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[2].getCANcoder().getDegrees()).getDegrees());
    SmartDashboard.putNumber("Mod 3 CAN Pose", Rotation2d.fromDegrees(mSwerveModules[3].getCANcoder().getDegrees()).getDegrees());

    odometry.update(gyro.getRotation2d(), getModulePositions());
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw()) :
      new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    mSwerveModules[0].setDesiredState(swerveModuleStates[0], isOpenLoop);
    mSwerveModules[1].setDesiredState(swerveModuleStates[1], isOpenLoop);
    mSwerveModules[2].setDesiredState(swerveModuleStates[2], isOpenLoop);
    mSwerveModules[3].setDesiredState(swerveModuleStates[3], isOpenLoop);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[]{
      mSwerveModules[0].getPosition(),
      mSwerveModules[1].getPosition(),
      mSwerveModules[2].getPosition(),
      mSwerveModules[3].getPosition()
    };
    return positions;
  }

  public static void resetGyro(){
    gyro.setYaw(270);
  }

  public Rotation2d getYaw(){
    new Rotation2d();
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return Constants.SwerveConstants.kinematics.toChassisSpeeds(
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    );
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, false);
  }

  public void configurePathPlanner(){
      RobotConfig config = new RobotConfig(74.088, 6.883, new ModuleConfig(0.048, 5.450, 1.200, DCMotor.getKrakenX60(1), 60, 1), Constants.SwerveConstants.kinematics.getModules());
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::driveRobotRelative,//driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[]{
      mSwerveModules[0].getState(),
      mSwerveModules[1].getState(),
      mSwerveModules[2].getState(),
      mSwerveModules[3].getState()
    };
    return states;
  }
}
