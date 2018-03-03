package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Gyro extends Subsystem {

	public double targetAngle;
	private double targetError;
	public double gyroKp = .007;
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void resetGyro() {
		Robot.imu.reset();
	}

	public double getGyroAngle() {

		return normalizeAngle(Robot.imu.getAngle());
	}

	public double GetTargetAngle() {
		return this.targetAngle;
	}

	public float getGyroYaw() {
		return Robot.imu.getYaw();
	}

	public double getGyroYawComp() {
		return (getGyroYaw() * gyroKp);
	}

	public void setTargetAngle(double targetAngle) {
		
		this.targetAngle = normalizeAngle(targetAngle);
	}
	
	public double getAngleError(double angle){
		angle = normalizeAngle(angle - getGyroAngle());
		
		if(angle<(-180))
    	{
    		angle += 360;
    	}
    	
    	if(angle>180)
    	{
    		angle -= 360;
    	}
		
		return angle;
	}
	
	public double normalizeAngle(double angle){
		while (angle > 360) {
			angle -= 360;
		}
		
		while (angle < 0){
			angle += 360;
		}
		
		return angle;
	}

	public double getTargetError() {
		targetError = targetAngle - getGyroAngle();
		if (Math.abs(targetError) < 180)
			return targetError;
		else if (targetError >= 180)
			return -360 + targetError;
		else
			return 360 + targetError;
	}
	
	public double getUnitCircleAngle(){
		return 360 - normalizeAngle(getGyroAngle() - 90);
	}
	
	public double toUnitCircleAngle(double angle){
		
		return 360 - normalizeAngle(angle - 90);
	}

	public double getTargetYawComp() {
		double maxError = .15;
		if ((getTargetError() * gyroKp) >= maxError) {
			return maxError;
		} else if (getTargetError() <= -maxError) {
			return -maxError;
		} else {
			return getTargetError() * gyroKp;
		}
	}

	public void updateStatus() {
		SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
		SmartDashboard.putNumber("Gyro Target Angle", GetTargetAngle());
//		SmartDashboard.putNumber("Gyro Error", getTargetYawComp());
//		SmartDashboard.putNumber("Gyro Target Error", getTargetError());
//		SmartDashboard.putNumber("Unit Circle Angle", getUnitCircleAngle());
	}
}
