package org.usfirst.frc.team171.robot.PIDsubsystems;

import org.usfirst.frc.team171.RobotParts.SwerveModule;
import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PositionWheel extends PIDSubsystem {

	private static final double Kp = .007;
	private static final double Ki = 0.0;
	private static final double Kd = 0.000;
	
	private SwerveModule mModule;
	public double highestOutput = 0;
	
    // Initialize your subsystem here
    public PositionWheel(SwerveModule mod) {
    	super("PositionWheel", Kp, Ki, Kd);
    	this.mModule = mod;
    	setSetpoint(0);
		getPIDController().setInputRange(0, 360);
		getPIDController().setContinuous();
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
        return mModule.getAbsEnc().getAngle();
    }
    
    public double getError(){
    	return getPIDController().getError();
    }
    
    public void setDesiredAngle(double angle){
    	setSetpoint(angle);
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
//    	double maxOutput = 1 - Math.abs(mModule.driveMotor.get());
//    	
//    	
////    	if(Math.abs(output)>highestOutput)
////    	{
////    		SmartDashboard.putNumber("max", Math.abs(output));
////    		highestOutput = Math.abs(output);
////    	}
//    	
//    	if(output > maxOutput)
//    	{
//    		output = maxOutput;
//    	}
//    	
//    	if(output < -maxOutput)
//    	{
//    		output = -maxOutput;
//    	}
    	
    	mModule.getDirMotor().set(output);
//    	SmartDashboard.putNumber("output", output);
    }
}
