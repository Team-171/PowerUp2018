package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WayPoint extends Command {

	private double m_targetX;
	private double m_targetY;
	private double m_targetAngle;
	private double waypointSpacing = 5;
	private double initialAngle;
	private double initialDistance;
	
	private SubWaypoint[] plotPath;
	private int numWaypoints;
	private int currentWaypoint = 0;
	
    public WayPoint(double targetX, double targetY, double targetAngle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	this.m_targetX = targetX;
    	this.m_targetY = targetY;
    	this.m_targetAngle = targetAngle;
    	this.initialAngle = Robot.gyro.getGyroAngle();
    	
    	this.initialDistance = Math.hypot((Robot.driveTrain.robotPosition()[0]-m_targetX), (Robot.driveTrain.robotPosition()[1]-m_targetY));
    	numWaypoints = (int) Math.ceil(initialDistance/waypointSpacing);
    	double angleToDestination = Robot.gyro.getAngleError(targetAngle);
    	double initialLocation[] = Robot.driveTrain.robotPosition();
    	
    	for(int i = 0; i<(numWaypoints-1); i++)
    	{
    		plotPath[i] = new SubWaypoint(initialLocation[0] + (Math.cos(Robot.gyro.toUnitCircleAngle(initialAngle+angleToDestination)) * (i * waypointSpacing)), initialLocation[1] + (Math.sin(Robot.gyro.toUnitCircleAngle(initialAngle+angleToDestination)) * (i * waypointSpacing)), (initialAngle + (angleToDestination * (i/numWaypoints))));
    	}
    	plotPath[(int) numWaypoints] = new SubWaypoint(m_targetX, m_targetY, m_targetAngle);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	plotPath[0].start();    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(plotPath[currentWaypoint].isFinished())
    	{
    		plotPath[currentWaypoint++].start();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return plotPath[numWaypoints].isFinished();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
