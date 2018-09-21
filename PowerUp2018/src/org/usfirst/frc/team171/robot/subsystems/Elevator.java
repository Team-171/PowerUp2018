package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.robot.Robot;
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
	private PWMTalonSRX m_liftMotorLeft;
	private PWMTalonSRX m_liftMotorRight;
	private AnalogPotentiometer m_liftPot;
	private double moveSpeed = .25;
	public PositionElevator elevatorPID;
	public boolean limitReached;
	private int maxTravel = 52;
	private double m_max = 33.5;
	private double m_min = 2;

	public Elevator(PWMTalonSRX liftMotorLeft, PWMTalonSRX liftMotorRight, AnalogInput liftPot) {
		this.m_liftMotorLeft = liftMotorLeft;
		this.m_liftMotorRight = liftMotorRight;
		this.m_liftPot = new AnalogPotentiometer(liftPot, maxTravel, 2.9);
		this.elevatorPID = new PositionElevator(this, m_max, m_min);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public double getElevatorPosition() {
		// TODO: add elevator control logic

		return maxTravel - this.m_liftPot.get();// 0;// m_liftEncoder.get();
	}

	// public void resetEncoder() {
	// m_liftEncoder.reset();
	// }

	/**
	 * 
	 * @param speed
	 *            elevator speed between -1 and 1
	 */
	public void moveElevator(double speed) {
		// TODO: check limit switches
//		if(speed < -0.5)
//		{
//			speed = -0.5;
//		}

		if (getElevatorPosition() > m_max && speed > 0) {
			speed = 0;
		}

		if (getElevatorPosition() < m_min && speed < 0) {
			speed = 0;
		}

		m_liftMotorLeft.set(speed);
		m_liftMotorRight.set(speed);
	}
	
	public double speed(){
		return this.m_liftMotorLeft.get();
	}

	public void moveElevatorManual(double speed) {

		if (Math.abs(speed) > 0) {
			elevatorPID.disable();
			this.moveElevator(speed);
		} else {
			if (!elevatorPID.getPIDController().isEnabled()) {
				elevatorPID.setSetpoint(this.getElevatorPosition());
				elevatorPID.enable();
			}
		}
	}

	public void updateStatus() {
//		SmartDashboard.putNumber("Elevator Speed", this.m_liftMotor.get());
		SmartDashboard.putNumber("Elevator thing", (Robot.elevator.getElevatorPosition() - m_min) / (m_max - m_min));
		SmartDashboard.putNumber("Elevator Height", getElevatorPosition());
//		SmartDashboard.putBoolean("Elevator PID Enabled", elevatorPID.getPIDController().isEnabled());
//		SmartDashboard.putNumber("Elevator PID Output", elevatorPID.getPIDController().get());
	}
}
