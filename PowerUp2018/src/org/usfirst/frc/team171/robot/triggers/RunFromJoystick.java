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
    	double deadband = .3;
    	
    	if(Robot.joystickRunning){
    		
    		
    		if(Robot.oneController)
    		{
        		Robot.elevator.moveElevatorManual(getOutput(deadband, 0.99, Robot.oi.gamepad.getRawAxis(2) - Robot.oi.gamepad.getRawAxis(3)));
    			if(Robot.oi.gamepad.getRawButton(6))
    			{
    				Robot.driveTrain.driveSwerve(getOutput(deadband, 0.99, Robot.oi.gamepad.getX()), getOutput(deadband, 0.99, -Robot.oi.gamepad.getY()), 0);
    				Robot.intake.runIntakeControlled(Robot.oi.gamepad);
    			}
    			else
    			{
    				Robot.driveTrain.driveSwerve(getOutput(deadband, 0.99, Robot.oi.gamepad.getX()), getOutput(deadband, 0.99, -Robot.oi.gamepad.getY()), getOutput(deadband, 0.99, Robot.oi.gamepad.getRawAxis(4)));
    				Robot.intake.runIntake(0);
    			}
    		}
    		else
    		{
    			Robot.driveTrain.driveSwerve(getOutput(deadband, 0.99, Robot.oi.gamepad.getX()), getOutput(deadband, 0.99, -Robot.oi.gamepad.getY()), getOutput(deadband, 0.99, Robot.oi.gamepad.getRawAxis(4)));
        		Robot.elevator.moveElevatorManual(getOutput(deadband, 0.99, -Robot.oi.operator_gamepad.getY()));
        		Robot.intake.runIntakeControlled(Robot.oi.operator_gamepad);
//        		Robot.intake.runIntake(getOutput(deadband, 0.99, (Robot.oi.operator_gamepad.getRawAxis(2) - Robot.oi.operator_gamepad.getRawAxis(3)) * 2), getOutput(deadband, 0.99, (Robot.oi.operator_gamepad.getRawAxis(2) - Robot.oi.operator_gamepad.getRawAxis(3)) * 2));
//        		Robot.intake.runFlipMotor(getOutput(deadband, 0.99, (Robot.oi.operator_gamepad.getRawAxis(2) - Robot.oi.operator_gamepad.getRawAxis(3)) * 2));
    		}
    		
    		
    		SmartDashboard.putNumber("VALUE", -Robot.oi.operator_gamepad.getRawAxis(5));
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
    
    public static double getOutput(double deadband, double maxOutput, double axis) {
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
