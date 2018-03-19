package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;
import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PlaceCubeLeftSwitch extends CommandGroup {

	public enum Direction {
		FRONT, SIDE, BACK
	}
	
    public PlaceCubeLeftSwitch(Direction direction) {
    	addParallel(new SetElevatorPosition(10));
    	switch (direction) {
    	case FRONT:
    		
    		break;
    	case SIDE:
    		addSequential(new WayPoint(65, 167.5, 90, 0.5));
    		break;
    	case BACK:
    		addSequential(new WayPoint(106, 167.5, 90, 0.5));
    		break;
    	}
    }
}
