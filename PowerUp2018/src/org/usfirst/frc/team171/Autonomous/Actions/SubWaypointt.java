package org.usfirst.frc.team171.Autonomous.Actions;
//package org.usfirst.frc.team171.Autonomous;
//
//import org.usfirst.frc.team171.robot.Robot;
//import org.usfirst.frc.team171.robot.PIDsubsystems.AutoMovementX;
//import org.usfirst.frc.team171.robot.PIDsubsystems.AutoMovementY;
//
//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
///**
// *
// */
//public class SubWaypointt extends Command {
//	
//	public static AutoMovementX PIDX;
//	public static AutoMovementY PIDY;
//	private double targetX;
//	private double targetY;
//	private double targetAngle;
//	private double speed;
//	
//	private double xOutput = 0;
//	private double yOutput = 0;
//	
//	private double inPositionDistance = 1;
//	private double initialDistance;
//	private double remainingDistance;
//	private double lastRemainingDistance;
//	private double initialAngle;
//	private double angleDifference;
////	private double maxOutput = .25;
//
//    public SubWaypointt(double m_targetX, double m_targetY, double m_targetAngle, double m_speed) {
//        this.targetX = m_targetX;
//    	this.targetY = m_targetY;
//    	this.targetAngle = m_targetAngle;
//    	this.speed = m_speed;
//        // Use requires() here to declare subsystem dependencies
//        // eg. requires(chassis);
//    }
//
//    // Called just before this Command runs the first time
//    protected void initialize() {
//    	Robot.gyro.gyroKp = 0.04;
//    	PIDX = new AutoMovementX(this, this.targetX);
//    	PIDX.getPIDController().setOutputRange(-speed, speed);
//    	PIDY = new AutoMovementY(this, this.targetY);
//    	PIDY.getPIDController().setOutputRange(-speed, speed);
//    	
//    	initialDistance = Math.hypot((Robot.driveTrain.robotPosition()[0]-targetX), (Robot.driveTrain.robotPosition()[1]-targetY));
//    	remainingDistance = initialDistance;
//    	lastRemainingDistance = initialDistance;
//    	initialAngle = Robot.gyro.getGyroAngle();
//    	
//    	angleDifference = Robot.gyro.getAngleError(targetAngle);
//    }
//
//    // Called repeatedly when this Command is scheduled to run
//    protected void execute() {
//    	remainingDistance = Math.hypot((Robot.driveTrain.robotPosition()[0]-targetX), (Robot.driveTrain.robotPosition()[1]-targetY));
//    	
//    	Robot.driveTrain.driveSwerve(PIDX.getPIDController().get(), PIDY.getPIDController().get(), 0);
//    	
//    	if(remainingDistance<lastRemainingDistance)
//    	{
//    		Robot.gyro.setTargetAngle(initialAngle + (angleDifference * (1 - (remainingDistance / initialDistance))));
//    	}
//    	
//    	lastRemainingDistance = remainingDistance;
//    }
//
//    // Make this return true when this Command no longer needs to run execute()
//    protected boolean isFinished() {
//        return remainingDistance<inPositionDistance;
//    }
//
//    // Called once after isFinished returns true
//    protected void end() {
//    	PIDX.disable();
//    	PIDY.disable();
//    	Robot.gyro.setTargetAngle(targetAngle);
//    	Robot.driveTrain.stopSwerve();
//    	Robot.gyro.gyroKp = 0.007;
//    	
//    }
//
//    // Called when another command which requires one or more of the same
//    // subsystems is scheduled to run
//    protected void interrupted() {
//    	PIDX.disable();
//    	PIDY.disable();
//    }
//    
//    protected void setXOutput(double output){
//    	this.xOutput = output;
//    }
//    
//    protected void setYOutput(double output){
//    	this.yOutput = output;
//    }
//}
