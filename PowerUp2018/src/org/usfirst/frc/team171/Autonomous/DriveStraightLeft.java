package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class DriveStraightLeft extends TimedCommand {

    public DriveStraightLeft() {
        super(4);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.driveSwerve(0, 0.75, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.driveSwerve(0, 0.75, 0);
    }

    // Called once after timeout
    protected void end() {
    	Robot.driveTrain.stopSwerve();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
