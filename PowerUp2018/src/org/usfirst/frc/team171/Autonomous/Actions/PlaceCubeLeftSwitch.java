package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;
import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;
import org.usfirst.frc.team171.robot.commands.SetIntake;
import org.usfirst.frc.team171.robot.commands.TimeDelay;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PlaceCubeLeftSwitch extends CommandGroup {

	public enum Direction {
		FRONT, SIDE, BACK
	}
	
	public PlaceCubeLeftSwitch(Direction direction) {
		this(direction, false);
	}
	
    public PlaceCubeLeftSwitch(Direction direction, boolean overrideClear) {
    	addParallel(new SetElevatorPosition(10));
    	switch (direction) {
    	case FRONT:
    		
    		break;
    		
    	case SIDE:
    		addSequential(new WayPoint(65, 167.5, 90, 0.6), 5);
    		addSequential(new SetIntake(null, -0.75));
    		addSequential(new TimeDelay(0.5));
    		addSequential(new SetIntake(null, 0));
    		break;
    		
    	case BACK:
    		if (!overrideClear) {
        		addSequential(new PlatformClearLeft(270));
    		}
    		addSequential(new WayPoint(106, 167.5, 90, 0.6));
    		addSequential(new SetIntake(null, -0.75));
    		addSequential(new TimeDelay(0.5));
    		addSequential(new SetIntake(null, 0));
    		break;
    	}
    }
}
