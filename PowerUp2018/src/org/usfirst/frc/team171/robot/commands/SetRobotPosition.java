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

    public SetRobotPosition(double fieldX, double fieldY) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		RobotMap.leftFrontSwerve.setFieldXY((robotWidth * Math.cos((Robot.gyro.getGyroAngle() + 135) / (Math.PI / 180))) + fieldX,
				(robotLength * Math.sin((Robot.gyro.getGyroAngle() + 135) / (Math.PI / 180))) + fieldY);
		RobotMap.leftBackSwerve.setFieldXY((robotWidth * Math.cos((Robot.gyro.getGyroAngle() + 225) / (Math.PI / 180))) + fieldX,
				(robotLength * Math.sin((Robot.gyro.getGyroAngle() + 225) / (Math.PI / 180))) + fieldY);
		RobotMap.rightFrontSwerve.setFieldXY((robotWidth * Math.cos((Robot.gyro.getGyroAngle() + 45) / (Math.PI / 180))) + fieldX,
				(robotLength * Math.sin((Robot.gyro.getGyroAngle() + 45) / (Math.PI / 180))) + fieldY);
		RobotMap.rightBackSwerve.setFieldXY((robotWidth * Math.cos((Robot.gyro.getGyroAngle() + 315) / (Math.PI / 180))) + fieldX,
				(robotLength * Math.sin((Robot.gyro.getGyroAngle() + 315) / (Math.PI / 180))) + fieldY);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
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
