package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.RobotMotion.RunFromJoystick;
import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public boolean fieldOriented = false;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new RunFromJoystick());
	}

	/*
	 * @param x x-axis movement, from -1.0 (left) to 1.0 (right)
	 * 
	 * @param y y-axis movement, from -1.0 (reverse) to 1.0 (forward)
	 * 
	 * @param rotation robot rotation, from -1.0 (CCW) to 1.0 (CW)
	 */
	public void driveSwerve(double x, double y, double rotation) {
		if (fieldOriented) {
			final double angle = Robot.gyro.getGyroAngle() * Math.PI / 180.0;
			final double temp = y * Math.cos(angle) + x * Math.sin(angle);
			x = -y * Math.sin(angle) + x * Math.cos(angle);
			y = temp;
		}

		final double LENGTH = 1.0;
		final double WIDTH = 1.0;
		final double RADIUS = Math.hypot(LENGTH, WIDTH);

		final double a = x - rotation * (LENGTH / RADIUS);
		final double b = x + rotation * (LENGTH / RADIUS);
		final double c = y - rotation * (WIDTH / RADIUS);
		final double d = y + rotation * (WIDTH / RADIUS);
		
		RobotMap.rightFrontSwerve.set((Math.atan2(b, c) * 180/Math.PI) + 180, Math.hypot(b, c));
		RobotMap.leftFrontSwerve.set((Math.atan2(b, d) * 180/Math.PI) + 180, Math.hypot(b, d));
		RobotMap.leftBackSwerve.set((Math.atan2(a, d) * 180/Math.PI) + 180, Math.hypot(a, d));
		RobotMap.rightBackSwerve.set((Math.atan2(a, c) * 180/Math.PI) + 180, Math.hypot(a, c));
	}

	public void driveSwervePolar(double angle, double magnitude, double rotation) {
		driveSwerve(Math.cos(angle * (Math.PI / 180)) * magnitude, Math.sin(angle * (Math.PI / 180)) * magnitude,
				rotation);
	}
}
