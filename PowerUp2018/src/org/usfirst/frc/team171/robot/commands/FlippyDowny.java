package org.usfirst.frc.team171.robot.commands;

import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class FlippyDowny extends TimedCommand {

	public FlippyDowny(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		RobotMap.intake.runFlipMotor(1);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Called once after timeout
	protected void end() {
		RobotMap.intake.runFlipMotor(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
