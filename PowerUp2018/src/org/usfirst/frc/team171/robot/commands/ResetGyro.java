package org.usfirst.frc.team171.robot.commands;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ResetGyro extends InstantCommand {

    public ResetGyro() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.gyro.resetGyro();
    }

}
