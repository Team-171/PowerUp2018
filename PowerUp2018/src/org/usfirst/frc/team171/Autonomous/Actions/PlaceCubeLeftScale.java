package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;
import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;
import org.usfirst.frc.team171.robot.commands.SetIntake;
import org.usfirst.frc.team171.robot.commands.TimeDelay;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PlaceCubeLeftScale extends CommandGroup {
	
	public PlaceCubeLeftScale() {
		this(false);
	}
	
	public PlaceCubeLeftScale(boolean overrideClear) {
		if (!overrideClear) {
			addSequential(new PlatformClearLeft(45));
		}
    	addParallel(new WayPoint(65, 280, 25, 0.5), 6);
    	addParallel(new SetElevatorPosition(10)); // TODO: Set actual position
		addSequential(new SetIntake(null, -0.75));
		addSequential(new TimeDelay(0.5));
		addSequential(new SetIntake(null, 0));
	}
}
