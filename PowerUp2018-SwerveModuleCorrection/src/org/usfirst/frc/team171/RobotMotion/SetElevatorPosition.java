package org.usfirst.frc.team171.RobotMotion;

import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;


/**
 *
 */
public class SetElevatorPosition extends Command {
	
	private double m_position;
	
    public SetElevatorPosition(double position) {
        this.m_position = position;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.elevator.elevatorPID.setSetpoint(this.m_position);
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
