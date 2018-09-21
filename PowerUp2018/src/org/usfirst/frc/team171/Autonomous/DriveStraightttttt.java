package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraightttttt extends Command {

    public DriveStraightttttt() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.setRobotPosition(new double[]{0, 0});
    	Robot.driveTrain.driveSwerve(0, 1, 0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.driveSwerve(0, 1, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Robot.driveTrain.robotPosition()[1] > 130;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stopSwerve();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
