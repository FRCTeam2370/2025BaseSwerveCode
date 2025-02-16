// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveModule;

/** Add your docs here. */
public class CTREModuleState {
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        System.out.println("current angle" + currentAngle.getDegrees());
        System.out.println("desired angle" + desiredState.angle.getDegrees());
    //double targetAngle = placeInAppropriate0To360Scope(SwerveModule.makePositiveDegrees(currentAngle.getDegrees()) - 180, SwerveModule.makePositiveDegrees(desiredState.angle.getDegrees()) - 180);
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    // targetAngle = SwerveModule.makePositiveDegrees(targetAngle);
    // System.out.println("make Pose 0to360" + targetAngle);
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
      double lowerBound;
      double upperBound;
      double lowerOffset = scopeReference % 360;
      if (lowerOffset >= 0) {
          lowerBound = scopeReference - lowerOffset;
          upperBound = scopeReference + (360 - lowerOffset);
      } else {
          upperBound = scopeReference - lowerOffset;
          lowerBound = scopeReference - (360 + lowerOffset);
      }
      while (newAngle < lowerBound) {
          newAngle += 360;
      }
      while (newAngle > upperBound) {
          newAngle -= 360;
      }
      if (newAngle - scopeReference > 180) {
          newAngle -= 360;
      } else if (newAngle - scopeReference < -180) {
          newAngle += 360;
      }
      System.out.println("PlaceIn0To360DegreeScope" + newAngle);
      return newAngle;
  }
}
