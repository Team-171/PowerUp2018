package org.usfirst.frc.team171.RobotMotion;

import org.usfirst.frc.team171.robot.subsystems.AbsoluteEncoder;
import org.usfirst.frc.team171.robot.subsystems.PositionWheel;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;

public class SwerveModule {

	public PWMTalonSRX driveMotor;
	public Encoder driveEncoder;
	public PWMTalonSRX directionMotor;
	public AbsoluteEncoder directionEncoder;
	public double speed;
	public double angle;
	private PositionWheel PIDController;
	private boolean reversed = false;
	
	public SwerveModule(PWMTalonSRX drive, Encoder driveEnc, PWMTalonSRX dir, AbsoluteEncoder dirEnc){
		this.driveMotor = drive;
		this.driveEncoder = driveEnc;
		this.directionMotor = dir;
		this.directionEncoder = dirEnc;
		this.PIDController = new PositionWheel(this);
	}
	
	public void set(double angle, double speed){
		setAngle(angle);
		setSpeed(speed);
	}
	
	public void setSpeed(double targetSpeed){
		if(targetSpeed > 1){
			targetSpeed = 1;
		}
		
		if(targetSpeed < -1){
			targetSpeed = -1;
		}
		
		if(reversed){
			targetSpeed = -targetSpeed;
		}
		
		speed = targetSpeed;
		driveMotor.set(targetSpeed);
	}
	
	public void setAngle(double targetAngle){
//		if(Math.abs(targetAngle - directionEncoder.getAngle())>90 && Math.abs(360 - targetAngle + directionEncoder.getAngle())>90)
//		{
//		    reversed = !reversed;
//			if(targetAngle>180)
//			{
//				targetAngle += 180;
//			}
//			else
//			{
//				targetAngle -= 180; 
//			}
//			driveMotor.set(-speed);
//		}
		angle = targetAngle;
		PIDController.setDesiredAngle(targetAngle);
	}
}
