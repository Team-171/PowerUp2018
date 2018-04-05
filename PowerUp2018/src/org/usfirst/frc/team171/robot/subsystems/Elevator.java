package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.robot.PIDsubsystems.PositionElevator;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Elevator extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private PWMTalonSRX m_liftMotor;
	private AnalogPotentiometer m_liftPot;
	private double moveSpeed = .25;
	public PositionElevator elevatorPID;
	public boolean limitReached;
	private int maxTravel = 52;

	public Elevator(PWMTalonSRX liftMotor, AnalogInput liftPot) {
		this.elevatorPID = new PositionElevator(this);
		this.m_liftMotor = liftMotor;
		this.m_liftPot = new AnalogPotentiometer(liftPot, maxTravel, 2.9);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public double getElevatorPosition() {
		// TODO: add elevator control logic

		return maxTravel - this.m_liftPot.get();// 0;// m_liftEncoder.get();
	}

//	public void resetEncoder() {
//		m_liftEncoder.reset();
//	}

	/**
	 * 
	 * @param speed
	 *            elevator speed between -1 and 1
	 */
	public void moveElevator(double speed) {
		// TODO: check limit switches

		m_liftMotor.set(speed);
	}

	public void moveElevatorManual(double speed) {

		if (Math.abs(speed) > 0) {
//			elevatorPID.disable();
			this.moveElevator(speed);
		} //else {
//			if (!elevatorPID.getPIDController().isEnabled()) {
//				elevatorPID.setSetpoint(this.getElevatorPosition());
//				elevatorPID.enable();
//			}
//
//		}
	}
	
	public void updateStatus(){
		SmartDashboard.putNumber("Elevator Speed", this.m_liftMotor.get());
	}
}
