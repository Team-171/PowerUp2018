package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;
import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PlaceCubeRightScale extends CommandGroup {
	
	public PlaceCubeRightScale() {
		this(false);
	}

    public PlaceCubeRightScale(boolean overrideClear) {
    	if (!overrideClear) {
        	addSequential(new PlatformClearRight(315));
    	}
    	addParallel(new WayPoint(259, 280, 335, 0.5));
    	addParallel(new SetElevatorPosition(10)); // TODO: Set actual position
    }
}
