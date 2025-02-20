// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveRobotRelative extends Command {
  private SwerveSubsystem mSwerve;
  private DoubleSupplier xSup, ySup, rotSup;
  private BooleanSupplier robotCentricSup;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(5);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(5);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(5);
  /** Creates a new TeleopSwerve. */
  public DriveRobotRelative(SwerveSubsystem mSwerve, DoubleSupplier xSup, DoubleSupplier ySup, DoubleSupplier rotSup, BooleanSupplier robotCentricSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    addRequirements(mSwerve);
    this.xSup = xSup;
    this.ySup = ySup;
    this.rotSup = rotSup;
    this.robotCentricSup = robotCentricSup;
    SwerveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVal = Math.abs(xSup.getAsDouble()) < 0.01 ? 0 : xSup.getAsDouble();
    double yVal = Math.abs(ySup.getAsDouble()) < 0.01 ? 0 : ySup.getAsDouble();
    double rotVal = Math.abs(rotSup.getAsDouble()) < 0.01 ? 0 : rotSup.getAsDouble();

    ChassisSpeeds speeds = new ChassisSpeeds(xLimiter.calculate(xVal) * Constants.SwerveConstants.maxSpeed, yLimiter.calculate(yVal) * Constants.SwerveConstants.maxSpeed, rotLimiter.calculate(rotVal * 0.2));

    mSwerve.driveRobotRelative(speeds);
  }
}