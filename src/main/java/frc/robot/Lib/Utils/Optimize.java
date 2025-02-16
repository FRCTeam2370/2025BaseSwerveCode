package frc.robot.Lib.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**find shortest path of rotation, assuming angles are from 0 to 360
 * <p>
 * This function Implements: Isaac (https://math.stackexchange.com/users/72/isaac), Shortest way to achieve target angle, URL (version: 2012-02-17): https://math.stackexchange.com/q/110236
 * 
 * @param target target angle in degrees [0, 360]
 * @param current current angle in degrees [0, 360]
 * @return double transformation to closest angle in degrees, positive clockwise, negative CC
 */
public class Optimize{
    public static double CalculateShortestPathOfRotation(double target, double current) {
        double alpha = target - current;
        double beta = alpha + 360.0;
        double gamma = alpha - 360.0; 
    
        double alphaAbs = Math.abs(alpha);
        double betaAbs = Math.abs(beta);
        double gammaAbs = Math.abs(gamma);
    
        // determine which value is shortest
        if (alphaAbs < betaAbs && alphaAbs < gammaAbs) return alpha;
        
        // code will not reach here unless: betaAbs < alphaAbs and gammaAbs < alphaAbs 
        else if (betaAbs < gammaAbs) return beta; 
        
        // code will not reach here unless: gammaAbs < betaAbs and gammaAbs < alphaAbs 
        else return gamma;
    }
    
    /**Minimize the change in heading(angle) for the desired swerve module state.
     * This will ensure that a steer motor is never going to rotate more than 90 degrees.
     * The returned SwerveModuleState's angle can be directly used as motor position.
     * 
     * @param desiredState the unoptimized state that is targeted 
     * @param currentSteerMotorAngle the current module's steer motor angle. (-∞, ∞)
     * @return SwerveModuleState the optimized state
     * @apiNote This is a custom implenmentation of the optimization algorithm. To fix WPILIB's implenmentation causing full 180 degree is certain circumstances.
     */
    public static SwerveModuleState SwerveOptimizeAngle(SwerveModuleState desiredState, Rotation2d currentSteerMotorAngle) {
        // convert any negative angles to positive angle equivalents 
        double desiredAngle = desiredState.angle.getDegrees();
        while (desiredAngle < 0) desiredAngle += 360.0; 
        desiredAngle = desiredAngle % 360.0;
    
        double currentAngle = currentSteerMotorAngle.getDegrees();
        while (currentAngle < 0) currentAngle += 360.0;
        currentAngle = currentAngle % 360.0;
    
        // calculate the other supplementary angle 
        double altAngle = (desiredAngle + 180.0) % 360.0;
    
        // find how to get to those 2 angles
        double transformToDesired = CalculateShortestPathOfRotation(desiredAngle, currentAngle);
        double transformToAlt = CalculateShortestPathOfRotation(altAngle, currentAngle);
        
        // find which of the 2 possible angles have a shorter travel distance
        double resTransform = transformToDesired; 
        // no need to invert direction if going with the already desired angle
        if (Math.abs(transformToAlt) < Math.abs(transformToDesired)) {
            resTransform = transformToAlt;
            desiredState.speedMetersPerSecond *= -1; // invert direction if using supplenmentry other angle
        }
    
        desiredState.angle = Rotation2d.fromDegrees(currentSteerMotorAngle.getDegrees() + resTransform);
        return desiredState;
    }
}
