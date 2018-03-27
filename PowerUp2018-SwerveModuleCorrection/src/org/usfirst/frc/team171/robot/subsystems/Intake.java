package org.usfirst.frc.team171.robot.subsystems;

import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Intake extends Subsystem {
	
	private PWMTalonSRX leftArm;
	private PWMTalonSRX rightArm;
	private double moveSpeed = 0.5;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Intake(PWMTalonSRX leftArm, PWMTalonSRX rightArm) {
		this.leftArm = leftArm;
		this.rightArm = rightArm;
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void runIntake(double leftValue, double rightValue) {
    	leftArm.set(leftValue * moveSpeed);
    	rightArm.set(-rightValue * moveSpeed);
    }
}

