package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WayPoint extends Command {

	private double m_targetX;
	private double m_targetY;
	private double waypointSpacing = 5;
	
	private SubWaypoint[] plotPath;
	
    public WayPoint(double targetX, double targetY) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	this.m_targetX = targetX;
    	this.m_targetY = targetY;
    	
    	double numWaypoints = Math.ceil(Math.hypot((Robot.driveTrain.robotPosition()[0]-m_targetX), (Robot.driveTrain.robotPosition()[1]-m_targetY))/waypointSpacing);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
