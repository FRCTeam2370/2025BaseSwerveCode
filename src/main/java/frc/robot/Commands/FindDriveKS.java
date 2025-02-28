// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindDriveKS extends Command {
  SwerveSubsystem mSwerve;
  double offset = 0;
  /** Creates a new FindDriveKS. */
  public FindDriveKS(SwerveSubsystem mSwerve) {
    this.mSwerve = mSwerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.driver.leftBumper().getAsBoolean() == true){
      offset += 0.001;
    }else if(RobotContainer.driver.rightBumper().getAsBoolean() == true){
      offset -= 0.001;
    }
    mSwerve.drive(new Translation2d(offset,0), 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
