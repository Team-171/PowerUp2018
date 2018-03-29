package org.usfirst.frc.team171.robot.commands;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetRobotPosition extends Command {

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
		Robot.driveTrain.setRobotPosition(new double[]{m_fieldX, m_fieldY});
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
