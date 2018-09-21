//package org.usfirst.frc.team171.Autonomous.Actions;
//
//import org.usfirst.frc.team171.robot.Robot;
//import org.usfirst.frc.team171.robot.PIDsubsystems.AutoMovementX;
//import org.usfirst.frc.team171.robot.PIDsubsystems.AutoMovementY;
//
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
//public class SubWaypoint {
//	
//	private double targetX;
//	private double targetY;
//	private double targetAngle;
//	private double speed;
//	
//	private double inPositionDistance;
//	private double initialDistance;
//	private double remainingDistance;
//	private double lastRemainingDistance;
//	private double initialAngle;
//	private double angleDifference;
//	
//	public SubWaypoint(double m_targetX, double m_targetY, double m_targetAngle, double m_speed, double inPositionDistance) {
//        this.targetX = m_targetX;
//    	this.targetY = m_targetY;
//    	this.targetAngle = m_targetAngle;
//    	this.speed = m_speed;
//    	this.inPositionDistance = inPositionDistance;
//        // Use requires() here to declare subsystem dependencies
//        // eg. requires(chassis);
//    }
//
//	public void start(){
//		WayPoint.PIDX.setWaypoint(this);
//		WayPoint.PIDY.setWaypoint(this);
//		
//		initialDistance = Math.hypot((targetX - Robot.driveTrain.robotPosition()[0]), (targetY - Robot.driveTrain.robotPosition()[1]));
//    	remainingDistance = initialDistance;
//    	lastRemainingDistance = initialDistance;
//    	initialAngle = Robot.gyro.getGyroAngle();
//    	
//    	angleDifference = Robot.gyro.getAngleError(targetAngle);
////    	SmartDashboard.putString("SubWaypoint", "X: " + this.targetX + "Y: " + this.targetY);
//	}
//	
//	public void run(){
//        remainingDistance = Math.hypot((targetX - Robot.driveTrain.robotPosition()[0]), (targetY - Robot.driveTrain.robotPosition()[1]));
//    	
////        SmartDashboard.putNumber("PIDx", WayPoint.PIDX.getPIDController().get());
////        SmartDashboard.putNumber("PIDy", WayPoint.PIDY.getPIDController().get());
//    	Robot.driveTrain.driveSwerve(WayPoint.PIDX.getPIDController().get(), WayPoint.PIDY.getPIDController().get(), 0);
//    	
//    	if(remainingDistance<lastRemainingDistance)
//    	{
//    		Robot.gyro.setTargetAngle(initialAngle + (angleDifference * (1 - (remainingDistance / initialDistance))));
//    	}
//    	
//    	lastRemainingDistance = remainingDistance;
//	}
//	
//	public boolean isFinished() {
//        return remainingDistance<inPositionDistance;
//    }
//	
//	public void end(){
//    	Robot.gyro.setTargetAngle(targetAngle);
////    	Robot.driveTrain.stopSwerve();
//    	
//	}
//	
//	public double getTargetX(){
//		return this.targetX;
//	}
//	
//	public double getTargetY(){
//		return this.targetY;
//	}
//	
//	public double getSpeed(){
//		return this.speed;
//	}
//	
//	public double getAngle(){
//		return this.targetAngle;
//	}
//}
