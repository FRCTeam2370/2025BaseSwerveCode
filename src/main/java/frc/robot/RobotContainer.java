// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveRobotRelative;
import frc.robot.Commands.GoToSwervePose;
import frc.robot.Commands.ResetGyro;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Subsystems.SwerveSubsystem;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);

  private final SwerveSubsystem mSwerve = new SwerveSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    //Put all NamedCommands here
    NamedCommands.registerCommand("Test", new ResetGyro(mSwerve));
    
    configureBindings();
  }

  private void configureBindings() {
    mSwerve.setDefaultCommand(new TeleopSwerve(mSwerve, ()-> driver.getRawAxis(0), ()-> -driver.getRawAxis(1), ()-> driver.getRawAxis(4), ()-> false));

    //driver.b().toggleOnTrue(new GoToSwervePose(mSwerve, new Pose2d(new Translation2d(11.4, 7.7), Rotation2d.fromDegrees(90))));

    driver.a().onTrue(new ResetGyro(mSwerve));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
