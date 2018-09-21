package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.triggers.RunFromJoystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intake extends Subsystem {
	
	private PWMTalonSRX m_leftArm;
	private PWMTalonSRX m_rightArm;
	private double moveSpeed = 1;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Intake(PWMTalonSRX leftArm, PWMTalonSRX rightArm) {
		this.m_leftArm = leftArm;
		this.m_rightArm = rightArm;
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void runIntake(double speed){
    	
    	runIntake(speed, speed);
    }
    
    public void runIntakeControlled(Joystick joystick){
    	double deadband = 0.3;
    	
    	double forwardSpeed = RunFromJoystick.getOutput(deadband, 0.99, joystick.getRawAxis(5));
    	double spinSpeed = -RunFromJoystick.getOutput(deadband, 0.99, joystick.getRawAxis(4));
    	
    	runIntake(forwardSpeed + spinSpeed, forwardSpeed - spinSpeed);
    	
//    	double leftSpeed = -RunFromJoystick.getOutput(deadband, 0.99, Robot.oi.operator_gamepad.getRawAxis(2));
//    	double rightSpeed = -RunFromJoystick.getOutput(deadband, 0.99, Robot.oi.operator_gamepad.getRawAxis(3));
//    	
//    	if(Robot.oi.operator_gamepad.getRawButton(5))
//    	{
//    		leftSpeed *= -1;
//    	}
//    	
//    	if(Robot.oi.operator_gamepad.getRawButton(6))
//    	{
//    		rightSpeed *= -1;
//    	}
    	
//    	runIntake(leftSpeed, rightSpeed);
    }
    
    public void runIntake(double leftValue, double rightValue) {
    	SmartDashboard.putNumber("Left", leftValue);
    	SmartDashboard.putNumber("Right", rightValue);
    	m_leftArm.set(-leftValue * moveSpeed);
    	m_rightArm.set(rightValue * moveSpeed);
    }
    
}

