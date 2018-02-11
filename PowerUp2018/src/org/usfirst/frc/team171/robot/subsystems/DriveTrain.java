package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.RobotMotion.RunFromJoystick;
import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public boolean fieldOriented = true;
	public boolean rotating = true;
	public boolean swerveDisabled = false;

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
		boolean angleOptimization = true;

		if (fieldOriented) {
			final double angle = Robot.gyro.getGyroAngle() * Math.PI / 180.0;
			final double temp = y * Math.cos(angle) + x * Math.sin(angle);
			x = -y * Math.sin(angle) + x * Math.cos(angle);
			y = temp;
		} else {
			// x *= .5;
		}

		if (rotating && rotation == 0.0) {
			rotating = false;
			Robot.gyro.setTargetAngle(Robot.gyro.getGyroAngle());
		}

		if (!rotating) {
			if (Math.abs(rotation) > 0.0) {
				rotating = true;
			} else {
				if (x != 0 && y != 0) {
					rotation = Robot.gyro.getTargetYawComp();
				}
			}
		}

		if (swerveDisabled && (x != 0 || y != 0 || rotation != 0)) {
			swerveDisabled = false;
			RobotMap.leftFrontSwerve.PIDController.enable();
			RobotMap.leftBackSwerve.PIDController.enable();
			RobotMap.rightFrontSwerve.PIDController.enable();
			RobotMap.rightBackSwerve.PIDController.enable();
		}

		if (!swerveDisabled && x == 0 && y == 0 && rotation == 0)
		{
			swerveDisabled = true;
			RobotMap.leftFrontSwerve.PIDController.disable();
			RobotMap.leftBackSwerve.PIDController.disable();
			RobotMap.rightFrontSwerve.PIDController.disable();
			RobotMap.rightBackSwerve.PIDController.disable();
		}

		SmartDashboard.putNumber("x", x);
		SmartDashboard.putNumber("y", y);
		SmartDashboard.putNumber("rotation", rotation);

		final double LENGTH = 1.0;
		final double WIDTH = 1.0;
		final double RADIUS = Math.hypot(LENGTH, WIDTH);

		final double a = x - rotation * (LENGTH / RADIUS);
		final double b = x + rotation * (LENGTH / RADIUS);
		final double c = y - rotation * (WIDTH / RADIUS);
		final double d = y + rotation * (WIDTH / RADIUS);

		RobotMap.rightFrontSwerve.set((Math.atan2(b, c) * (180 / Math.PI)), Math.hypot(b, c), angleOptimization);
		RobotMap.leftFrontSwerve.set((Math.atan2(b, d) * (180 / Math.PI)), Math.hypot(b, d), angleOptimization);
		RobotMap.leftBackSwerve.set((Math.atan2(a, d) * (180 / Math.PI)), Math.hypot(a, d), angleOptimization);
		RobotMap.rightBackSwerve.set((Math.atan2(a, c) * (180 / Math.PI)), Math.hypot(a, c), angleOptimization);

		// SmartDashboard.putData(Robot.imu.setName(subsystem, name););

		// SmartDashboard.putNumber("error",
		// RobotMap.leftFrontSwerve.PIDController.getPIDController().get());
		SmartDashboard.putNumber("Target Angle", (Math.atan2(b, c) * 180 / Math.PI));
	}

	public void driveSwervePolar(double angle, double magnitude, double rotation) {
		driveSwerve(Math.cos(angle * (Math.PI / 180)) * magnitude, Math.sin(angle * (Math.PI / 180)) * magnitude,
				rotation);
	}

	public double[] robotPosition() {
		return new double[] {
				(RobotMap.leftFrontSwerve.fieldX + RobotMap.leftBackSwerve.fieldX + RobotMap.rightFrontSwerve.fieldX
						+ RobotMap.rightBackSwerve.fieldX) / 4,
				(RobotMap.leftFrontSwerve.fieldY + RobotMap.leftBackSwerve.fieldY + RobotMap.rightFrontSwerve.fieldY
						+ RobotMap.rightBackSwerve.fieldY) / 4 };

	}
}
