package org.usfirst.frc.team171.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {
	
	private PWMTalonSRX m_leftArm;
	private PWMTalonSRX m_rightArm;
	private PWMTalonSRX m_flipMotor;
	private double moveSpeed = 0.5;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Intake(PWMTalonSRX leftArm, PWMTalonSRX rightArm, PWMTalonSRX flipMotor) {
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
    
    public void runIntake(double leftValue, double rightValue) {
    	m_leftArm.set(leftValue * moveSpeed);
    	m_rightArm.set(-rightValue * moveSpeed);
    }
    
    public void runFlipMotor(double speed){
    	this.m_flipMotor.set(speed);
    }
}

