package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.RobotMotion.SwerveModule;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class PositionWheel extends PIDSubsystem {

	private static final double Kp = .01;
	private static final double Ki = 0.0;
	private static final double Kd = 0.0;
	
	private SwerveModule mModule;
	
    // Initialize your subsystem here
    public PositionWheel(SwerveModule mod) {
    	super("PositionWheel", Kp, Ki, Kd);
    	this.mModule = mod;
    	setSetpoint(0);
		getPIDController().setContinuous();
		getPIDController().setInputRange(0, 360);
		getPIDController().setPercentTolerance(1);
		getPIDController().setOutputRange(-1, 1);
    	enable();
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }
    
    public void setPIDF(double Kp, double Ki, double Kd, double Kf) {
		getPIDController().setPID(Kp, Ki, Kd, Kf);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return mModule.directionEncoder.getAngle();
    }
    
    public void setDesiredAngle(double angle){
    	setSetpoint(angle);
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	mModule.directionMotor.set(output);
    }
}
