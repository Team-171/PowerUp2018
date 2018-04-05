package org.usfirst.frc.team171.robot.triggers;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RunFromJoystick extends Command {

    public RunFromJoystick() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double deadband = .2;
    	
    	if(Robot.joystickRunning){
    		Robot.driveTrain.driveSwerve(getOutput(deadband, 0.99, Robot.oi.gamepad.getX()), getOutput(deadband, 0.99, -Robot.oi.gamepad.getY()), getOutput(deadband, 0.99, Robot.oi.gamepad.getRawAxis(4)));
//    		RobotMap.elevator.moveElevatorManual(Robot.oi.gamepad.getRawAxis(2) - Robot.oi.gamepad.getRawAxis(3));
    		RobotMap.elevator.moveElevatorManual(getOutput(deadband, 0.99, -Robot.oi.operator_gamepad.getRawAxis(5)));
    		RobotMap.intake.runIntake(getOutput(deadband, 0.99, (Robot.oi.operator_gamepad.getRawAxis(2) - Robot.oi.operator_gamepad.getRawAxis(3)) * 2), getOutput(deadband, 0.99, (Robot.oi.operator_gamepad.getRawAxis(2) - Robot.oi.operator_gamepad.getRawAxis(3)) * 2));
    		RobotMap.intake.runFlipMotor(getOutput(deadband, 0.99, Robot.oi.operator_gamepad.getY()));
    	}
    	
//    	SmartDashboard.putNumber("X", getOutput(deadband, 1, Robot.oi.gamepad.getX()));
//    	SmartDashboard.putNumber("Y", getOutput(deadband, 1, Robot.oi.gamepad.getY()));
//    	SmartDashboard.putNumber("Rotation", getOutput(deadband, 1, Robot.oi.gamepad.getRawAxis(4)));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
    protected void interrupted() {
    }
    
    public double getOutput(double deadband, double maxOutput, double axis) {
		double output;
		if (Math.abs(axis) < deadband) {
			output = 0;
		} else {
			double motorOutput = (((Math.abs(axis) - deadband) / (1 - deadband)) * (axis / Math.abs(axis)));
			output = motorOutput * maxOutput;
		}
		return output;
	}
}
