package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;
import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PlaceCubeRightSwitch extends CommandGroup {
	
	public enum Direction {
		FRONT, SIDE, BACK
	}
	
	public PlaceCubeRightSwitch(Direction direction) {
		this(direction, false);
	}

    public PlaceCubeRightSwitch(Direction direction, boolean overrideClear) {
    	addParallel(new SetElevatorPosition(10));
    	switch (direction) {
    	case FRONT:
    		
    		break;
    		
    	case SIDE:
    		addSequential(new WayPoint(259, 167.5, 270, 0.5));
    		break;
    		
    	case BACK:
    		if (!overrideClear) {
        		addSequential(new PlatformClearRight(90));
    		}
    		addSequential(new WayPoint(218.5, 167.5, 180, 0.5));
    		break;
    	}
    }
}
