package org.usfirst.frc.team171.robot.triggers;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunLEDs extends Command {
	private final double minHeight = 2;
	private final double maxHeight = 30;
	private double timeStamp = 0.0;

	public RunLEDs() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.leds);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.driveTrain.isMoving()) {
//			if (Robot.elevator.speed() > 0) {
//				Robot.leds.setMode(LEDs.Mode.HEIGHT_UP);
//			} else {
//				Robot.leds.setMode(LEDs.Mode.HEIGHT_DOWN);
//			}

			Robot.leds.setDutyCycle((Robot.elevator.getElevatorPosition() - minHeight) / (maxHeight));

			timeStamp = Timer.getFPGATimestamp();
		} else {
			if ((Timer.getFPGATimestamp() - timeStamp) > 5) {
				Robot.leds.setMode(LEDs.Mode.IDLE);
			}
		}

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
}
