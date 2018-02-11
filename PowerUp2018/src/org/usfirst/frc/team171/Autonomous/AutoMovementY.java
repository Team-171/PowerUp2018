package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class AutoMovementY extends PIDSubsystem {

	private static final double Kp = .007;
	private static final double Ki = 0.0;
	private static final double Kd = 0.000;
	
	private SubWaypoint m_wayPoint;
	
    // Initialize your subsystem here
    public AutoMovementY(SubWaypoint wayPoint, double target) {
    	super("AutoMovementY", Kp, Ki, Kd);
    	setSetpoint(target);
    	m_wayPoint = wayPoint;
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return Robot.driveTrain.robotPosition()[1];
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	m_wayPoint.setYOutput(output);
    }
}