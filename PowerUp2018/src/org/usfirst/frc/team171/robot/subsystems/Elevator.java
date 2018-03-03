package org.usfirst.frc.team171.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private VictorSP m_liftMotor;
	private Encoder m_liftEncoder;
	private double moveSpeed = .25;
	
	public Elevator(VictorSP liftMotor, Encoder liftEncoder){
		this.m_liftMotor = liftMotor;
		this.m_liftEncoder = liftEncoder;
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    
    public void resetEncoder(){
    	m_liftEncoder.reset();
    }
    
    
    /**
     * 
     * @param moveUp true for up false for down
     */
    private void moveElevator(double speed){
    	//TODO: check limit switches
        m_liftMotor.set(speed);
    }
}

