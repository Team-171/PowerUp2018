package org.usfirst.frc.team171.robot.PIDsubsystems;

import org.usfirst.frc.team171.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class PositionElevator extends PIDSubsystem {
	
	private static final double Kp = 0.0;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;
	
	private Elevator m_elevator;

    // Initialize your subsystem here
    public PositionElevator(Elevator elevator) {
    	super("PositionElevator", Kp, Ki, Kd);
    	this.m_elevator = elevator;
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.

		getPIDController().setContinuous();
		getPIDController().setInputRange(0, 100); //Todo: Set range to inches
		getPIDController().setPercentTolerance(1);
		getPIDController().setOutputRange(-.75, .75);
    	enable();
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
