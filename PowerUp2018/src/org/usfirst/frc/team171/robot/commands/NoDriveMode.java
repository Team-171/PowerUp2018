package org.usfirst.frc.team171.robot.commands;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class NoDriveMode extends InstantCommand {

    public NoDriveMode() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.joystickRunning = true;
    	
    	RobotMap.leftFrontSwerve.getPID().enable();
    	RobotMap.leftBackSwerve.getPID().enable();
    	RobotMap.rightFrontSwerve.getPID().enable();
    	RobotMap.rightBackSwerve.getPID().enable();
    }

}
