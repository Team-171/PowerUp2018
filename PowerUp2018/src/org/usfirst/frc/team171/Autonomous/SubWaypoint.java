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
	private double targetAngle;
	
	private double xOutput = 0;
	private double yOutput = 0;
	
	private double inPositionDistance = 1;
	private double initialDistance;
	private double remainingDistance;
	private double lastRemainingDistance;
	private double initialAngle;
	private double angleDifference;
	private double maxOutput = .25;

    public SubWaypoint(double m_targetX, double m_targetY, double m_targetAngle) {
        this.targetX = m_targetX;
    	this.targetY = m_targetY;
    	this.targetAngle = m_targetAngle;
    	
    	
    	
    	
    	
    	
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.joystickRunning = false;
    	Robot.gyro.gyroKp = 0.04;
    	PIDX = new AutoMovementX(this, this.targetX);
    	PIDX.getPIDController().setOutputRange(-maxOutput, maxOutput);
    	PIDY = new AutoMovementY(this, this.targetY);
    	PIDY.getPIDController().setOutputRange(-maxOutput, maxOutput);
    	
    	initialDistance = Math.hypot((Robot.driveTrain.robotPosition()[0]-targetX), (Robot.driveTrain.robotPosition()[1]-targetX));
    	remainingDistance = initialDistance;
    	lastRemainingDistance = initialDistance;
    	initialAngle = Robot.gyro.getGyroAngle();
    	
    	angleDifference = Robot.gyro.getAngleError(targetAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	remainingDistance = Math.hypot((Robot.driveTrain.robotPosition()[0]-targetX), (Robot.driveTrain.robotPosition()[1]-targetY));
    	
    	Robot.driveTrain.driveSwerve(PIDX.getPIDController().get(), PIDY.getPIDController().get(), 0);
    	
    	if(remainingDistance<lastRemainingDistance)
    	{
    		Robot.gyro.setTargetAngle(initialAngle + (angleDifference * (remainingDistance / initialDistance)));
    	}
    	
    	lastRemainingDistance = remainingDistance;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return remainingDistance<inPositionDistance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	PIDX.disable();
    	PIDY.disable();
    	Robot.gyro.setTargetAngle(targetAngle);
    	Robot.driveTrain.stopSwerve();
    	Robot.joystickRunning = true;
    	Robot.gyro.gyroKp = 0.007;
    	
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
