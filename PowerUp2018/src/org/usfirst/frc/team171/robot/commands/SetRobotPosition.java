package org.usfirst.frc.team171.robot.commands;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetRobotPosition extends Command {

	private double robotWidth = 22.5;
	private double robotLength = 22.5;
	private double m_fieldX;
	private double m_fieldY;

	public SetRobotPosition(double fieldX, double fieldY) {
		this.m_fieldX = fieldX;
		this.m_fieldY = fieldY;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

		// RobotMap.leftFrontSwerve.setFieldXY((robotWidth *
		// Math.cos(Math.toRadians(135 + Robot.gyro.getUnitCircleAngle()))), 0);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.leftFrontSwerve.setFieldXY(
				(robotWidth * Math.cos(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 135))) + m_fieldX,
				(robotLength * Math.sin(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 135))) + m_fieldY);
		RobotMap.leftBackSwerve.setFieldXY(
				(robotWidth * Math.cos(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 225))) + m_fieldX,
				(robotLength * Math.sin(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 225))) + m_fieldY);
		RobotMap.rightFrontSwerve.setFieldXY(
				(robotWidth * Math.cos(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 45))) + m_fieldX,
				(robotLength * Math.sin(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 45))) + m_fieldY);
		RobotMap.rightBackSwerve.setFieldXY(
				(robotWidth * Math.cos(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 315))) + m_fieldX,
				(robotLength * Math.sin(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 315))) + m_fieldY);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
