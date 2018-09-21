package org.usfirst.frc.team171.robot.PIDsubsystems;

import org.usfirst.frc.team171.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class PositionElevator extends PIDSubsystem {
	
	private static final double Kp = 0.25;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;
	
	private Elevator m_elevator;
	private double m_max;
	private double m_min;

    // Initialize your subsystem here
    public PositionElevator(Elevator elevator, double max, double min) {
    	super("PositionElevator", Kp, Ki, Kd);
    	this.m_elevator = elevator;
    	this.m_max = max;
    	this.m_min = min;
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.

		getPIDController().setInputRange(m_min, m_max); //Todo: Set range to inches
//		getPIDController().setContinuous();
		getPIDController().setPercentTolerance(1);
		getPIDController().setOutputRange(-.5, 1);
		disable();
//    	enable();
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return m_elevator.getElevatorPosition();
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	m_elevator.moveElevator(output);
    }
}
