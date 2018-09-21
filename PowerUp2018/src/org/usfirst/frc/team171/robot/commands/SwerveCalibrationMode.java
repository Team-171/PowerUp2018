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
    	RobotMap.leftFrontSwerve.getPID().disable();
    	RobotMap.leftBackSwerve.getPID().disable();
    	RobotMap.rightFrontSwerve.getPID().disable();
    	RobotMap.rightBackSwerve.getPID().disable();
    	
    	RobotMap.leftFrontSwerve.set(0, .1, false);
    	RobotMap.leftBackSwerve.set(0, .1, false);
    	RobotMap.rightFrontSwerve.set(0, .1, false);
    	RobotMap.rightBackSwerve.set(0, .1, false);
    }

}
