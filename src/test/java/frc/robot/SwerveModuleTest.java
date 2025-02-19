package frc.robot;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleTest {

	@Test
	public void testOptimizeTurn() {
		SwerveModule module = new SwerveModule(0, Constants.FLConstants.FLConstants);
		System.out.println("Testing optimizeTurn()");

		// Test no-ops
		// Assertions.assertEquals(
		// 		new Rotation2d(Math.toRadians(0)),
		// 		module.optimizeTurn(new Rotation2d(Math.toRadians(0)),
		// 				new Rotation2d(Math.toRadians(0))));
		// Assertions.assertEquals(
		// 		new Rotation2d(Math.toRadians(0)),
		// 		module.optimizeTurn(new Rotation2d(Math.toRadians(180)),
		// 				new Rotation2d(Math.toRadians(180))));
		// Assertions.assertEquals(
		// 		new Rotation2d(Math.toRadians(0)),
		// 		module.optimizeTurn(new Rotation2d(Math.toRadians(-90)),
		// 				new Rotation2d(Math.toRadians(-90))));

		// Test turns
		// Assertions.assertEquals(
		// 		new Rotation2d(Math.toRadians(180)),
		// 		module.optimizeTurn(new Rotation2d(Math.toRadians(0)),
		// 				new Rotation2d(Math.toRadians(180))));
		// Assertions.assertEquals(
		// 		new Rotation2d(Math.toRadians(90)),
		// 		module.optimizeTurn(new Rotation2d(Math.toRadians(0)),
		// 				new Rotation2d(Math.toRadians(90))));
		// Assertions.assertEquals(
		// 		new Rotation2d(Math.toRadians(-90)),
		// 		module.optimizeTurn(new Rotation2d(Math.toRadians(0)),
		// 				new Rotation2d(Math.toRadians(-90))));
		// Assertions.assertEquals(
		// 		new Rotation2d(Math.toRadians(179)),
		// 		module.optimizeTurn(new Rotation2d(Math.toRadians(0)),
		// 				new Rotation2d(Math.toRadians(-181))));

		// Test angles > 360
		// Assertions.assertEquals(
		// 		new Rotation2d(Math.toRadians(-90)),
		// 		module.optimizeTurn(new Rotation2d(Math.toRadians(0)),
		// 				new Rotation2d(Math.toRadians(-810))));
	}
}