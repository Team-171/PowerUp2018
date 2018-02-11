package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SubWaypoint extends Command {
	
	public AutoMovementX PIDX;
	public AutoMovementY PIDY;
	private double targetX;
	private double targetY;
	
	private double xOutput = 0;
	private double yOutput = 0;
	
	private double inPositionDistance = 1;

    public SubWaypoint(double m_targetX, double m_targetY) {
        targetX = m_targetX;
    	targetY = m_targetY;
    	
    	PIDX = new AutoMovementX(this, targetX);
    	PIDY = new AutoMovementY(this, targetY);
    	
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.driveSwerve(xOutput, yOutput, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.hypot((Robot.driveTrain.robotPosition()[0]-targetX), (Robot.driveTrain.robotPosition()[1]-targetY))<inPositionDistance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	PIDX.disable();
    	PIDY.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	PIDX.disable();
    	PIDY.disable();
    }
    
    protected void setXOutput(double output){
    	this.xOutput = output;
    }
    
    protected void setYOutput(double output){
    	this.yOutput = output;
    }
}
