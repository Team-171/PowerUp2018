package org.usfirst.frc.team171.robot.commands;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SwerveCalibrationMode extends InstantCommand {

    public SwerveCalibrationMode() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.joystickRunning = false;
    	RobotMap.leftFrontSwerve.PIDController.disable();
    	RobotMap.leftBackSwerve.PIDController.disable();
    	RobotMap.rightFrontSwerve.PIDController.disable();
    	RobotMap.rightBackSwerve.PIDController.disable();
    	
    	RobotMap.leftFrontSwerve.set(0, .1, false);
    	RobotMap.leftBackSwerve.set(0, .1, false);
    	RobotMap.rightFrontSwerve.set(0, .1, false);
    	RobotMap.rightBackSwerve.set(0, .1, false);
    }

}
