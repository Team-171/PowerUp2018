package org.usfirst.frc.team171.robot.commands;

import org.usfirst.frc.team171.robot.RobotMap;
import org.usfirst.frc.team171.robot.triggers.WaitUntil;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetIntake extends Command {

	private WaitUntil m_wait;
	private double m_speed;
	
    public SetIntake(WaitUntil wait, double speed) {
    	this.m_wait = wait;
    	this.m_speed = speed;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(this.m_wait!=null)
    	{
    		if(m_wait.inTolerance())
    		{
    			RobotMap.intake.runIntake(m_speed);
    		}
    	}
    	else
    	{
    		RobotMap.intake.runIntake(m_speed);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(this.m_wait == null)
    	{
    		return true;
    	}
    	else
    	{
    		return this.m_wait.inTolerance();
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
