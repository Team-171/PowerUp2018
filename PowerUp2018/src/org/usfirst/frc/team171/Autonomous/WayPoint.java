package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	private double m_speed;
	
	private SubWaypoint[] plotPath;
	private int numWaypoints;
	private int currentWaypoint = 0;
	
    public WayPoint(double targetX, double targetY, double targetAngle, double speed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	
    	this.m_targetX = targetX;
    	this.m_targetY = targetY;
    	this.m_targetAngle = targetAngle;
    	this.m_speed = speed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	this.initialAngle = Robot.gyro.getGyroAngle();
    	double xDistance = Robot.driveTrain.robotPosition()[0]-m_targetX;
    	double yDistance = Robot.driveTrain.robotPosition()[1]-m_targetY;
    	this.initialDistance = Math.hypot(xDistance, yDistance);
    	
    	numWaypoints = (int) Math.ceil(initialDistance/waypointSpacing);
    	SmartDashboard.putNumber("NumWaypoints", numWaypoints);
    	double angleToDestination = Robot.gyro.getAngleError(m_targetAngle);
    	double initialLocation[] = Robot.driveTrain.robotPosition();
    	
    	plotPath = new SubWaypoint[numWaypoints];
    	
    	for(int i = 0; i<(numWaypoints-1); i++)
    	{
    		double xAddition = (xDistance/initialDistance) * (i * waypointSpacing);
    		double yAddition = (yDistance/initialDistance) * (i * waypointSpacing);
    		double angleAddition = angleToDestination * (i/numWaypoints);
    		SmartDashboard.putNumber("x" + i, xAddition);
    		
    		plotPath[i] = new SubWaypoint((initialLocation[0] + xAddition), (initialLocation[1] + yAddition), (initialAngle + angleAddition), m_speed);
    	}
    	plotPath[numWaypoints] = new SubWaypoint(m_targetX, m_targetY, m_targetAngle, m_speed);
    	
    	plotPath[0].start();    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putBoolean("Waypoint Running", plotPath[0].isRunning());
    	
    	if(!plotPath[currentWaypoint].isRunning())
    	{
    		plotPath[++currentWaypoint].start();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;//!plotPath[numWaypoints].isRunning();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
