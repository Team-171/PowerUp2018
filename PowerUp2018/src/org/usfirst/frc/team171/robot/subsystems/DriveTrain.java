package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;
import org.usfirst.frc.team171.robot.triggers.RunFromJoystick;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public boolean fieldOriented = true;
	public boolean rotating = false;
	public boolean swerveDisabled = false;
	
	 private final double[] ws = new double[4];

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

		if (rotating && rotation == 0.0) {
			rotating = false;
			Robot.gyro.setTargetAngle(Robot.gyro.getGyroAngle());
		}

		if (!rotating) {
			if (Math.abs(rotation) > 0.0) {
				rotating = true;
			} else {
				if (x != 0 || y != 0) {
					rotation = Robot.gyro.getTargetYawComp();
				}
			}
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
		
		ws[0] = Math.hypot(b, d);
		ws[1] = Math.hypot(a, d);
		ws[2] = Math.hypot(b, c);
		ws[3] = Math.hypot(a, c);
		
		final double maxSpeed = 1;
		final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
		if(maxWheelSpeed>maxSpeed)
		{
			for (int i = 0; i < 4; i++) {
		        ws[i] /= maxWheelSpeed;
		      }
		}

		RobotMap.leftFrontSwerve.set((Math.atan2(b, d) * 180 / Math.PI), ws[0], angleOptimization);
		RobotMap.leftBackSwerve.set((Math.atan2(a, d) * 180 / Math.PI), ws[1], angleOptimization);
		RobotMap.rightFrontSwerve.set((Math.atan2(b, c) * 180 / Math.PI), ws[2], angleOptimization);
		RobotMap.rightBackSwerve.set((Math.atan2(a, c) * 180 / Math.PI), ws[3], angleOptimization);

		// SmartDashboard.putData(Robot.imu.setName(subsystem, name););

		// SmartDashboard.putNumber("error",
		// RobotMap.leftFrontSwerve.PIDController.getPIDController().get());
		SmartDashboard.putNumber("Target Angle", (Math.atan2(b, c) * 180 / Math.PI));
	}

	public void driveSwervePolar(double angle, double magnitude, double rotation) {
		driveSwerve(Math.cos(angle * (Math.PI / 180)) * magnitude, Math.sin(angle * (Math.PI / 180)) * magnitude,
				rotation);
	}
	
	public void stopSwerve(){
		driveSwerve(0, 0, 0);
	}

	public double[] robotPosition() {
		return new double[] {
				(RobotMap.leftFrontSwerve.fieldX + RobotMap.leftBackSwerve.fieldX + RobotMap.rightFrontSwerve.fieldX
						+ RobotMap.rightBackSwerve.fieldX) / 4,
				(RobotMap.leftFrontSwerve.fieldY + RobotMap.leftBackSwerve.fieldY + RobotMap.rightFrontSwerve.fieldY
						+ RobotMap.rightBackSwerve.fieldY) / 4 };

	}
	
	public void updateStatus(){
//		SmartDashboard.putNumber("Front Left Position X", RobotMap.leftFrontSwerve.fieldX);
//		SmartDashboard.putNumber("Front Left Position Y", RobotMap.leftFrontSwerve.fieldY);
//		
//		SmartDashboard.putNumber("Back Left Position X", RobotMap.leftBackSwerve.fieldX);
//		SmartDashboard.putNumber("Back Left Position Y", RobotMap.leftBackSwerve.fieldY);
//		
//		SmartDashboard.putNumber("Front Right Position X", RobotMap.rightFrontSwerve.fieldX);
//		SmartDashboard.putNumber("Front Right Position Y", RobotMap.rightFrontSwerve.fieldY);
//		
//		SmartDashboard.putNumber("Back Right Position X", RobotMap.rightBackSwerve.fieldX);
//		SmartDashboard.putNumber("Back Right Position Y", RobotMap.rightBackSwerve.fieldY);
		
		SmartDashboard.putNumber("Robot Position X", robotPosition()[0]);
		SmartDashboard.putNumber("Robot Position Y", robotPosition()[1]);
		
//		SmartDashboard.putNumber("Encoder", RobotMap.leftFrontSwerve.driveEncoder.get());
		
//		SmartDashboard.putNumber("Front Left Test Angle", RobotMap.leftFrontSwerve.testAng);
//		SmartDashboard.putNumber("Back Left Test Angle", RobotMap.leftBackSwerve.testAng);
//		SmartDashboard.putNumber("Front Right Test Angle", RobotMap.rightFrontSwerve.testAng);
//		SmartDashboard.putNumber("Back Right Test Angle", RobotMap.rightBackSwerve.testAng);
	}
}
