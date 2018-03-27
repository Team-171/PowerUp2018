package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.robot.PIDsubsystems.PositionElevator;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Elevator extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	private Spark m_liftMotor;
	private Encoder m_liftEncoder;
	private double moveSpeed = .25;
	public PositionElevator elevatorPID;
	public boolean limitReached;
	
	public Elevator(Spark liftMotor, Encoder liftEncoder){
		this.elevatorPID = new PositionElevator(this);
		this.m_liftMotor = liftMotor;
		this.m_liftEncoder = liftEncoder;
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public double getElevatorPosition() {
    	//Todo: add elevator control logic
    	
    	return 0;//m_liftEncoder.get();
    }
    
    public void resetEncoder(){
    	m_liftEncoder.reset();
    }
    
    
    /**
     * 
     * @param speed elevator speed between -1 and 1
     */
    public void moveElevator(double speed){
    	//TODO: check limit switches
    	
        m_liftMotor.set(speed);
    }
    
    public void moveElevatorManual(double speed) {
    	
    	if (speed > 0) {	
        	elevatorPID.disable();
        	this.moveElevator(speed);
    	} else {
    		elevatorPID.setSetpoint(this.getElevatorPosition());
    		elevatorPID.enable();
    	}
    }
}

