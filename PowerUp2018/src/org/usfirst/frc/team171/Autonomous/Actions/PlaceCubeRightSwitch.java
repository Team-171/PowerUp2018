package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;
import org.usfirst.frc.team171.robot.commands.SetIntake;
import org.usfirst.frc.team171.robot.commands.TimeDelay;

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
    		addSequential(new WayPoint(215, 120, 0, 0.6), 5);
    		addSequential(new SetIntake(null, -0.75));
    		addSequential(new TimeDelay(0.5));
    		addSequential(new SetIntake(null, 0));
    		break;
    		
    	case SIDE:
    		addSequential(new WayPoint(259, 167.5, 270, 0.5), 5);
    		addSequential(new SetIntake(null, -0.75));
    		addSequential(new TimeDelay(0.5));
    		addSequential(new SetIntake(null, 0));
    		break;
    		
    	case BACK:
    		if (!overrideClear) {
        		addSequential(new PlatformClearRight(90));
    		}
    		addSequential(new WayPoint(218.5, 167.5, 180, 0.5));
    		addSequential(new SetIntake(null, -0.75));
    		addSequential(new TimeDelay(0.5));
    		addSequential(new SetIntake(null, 0));
    		break;
    	}
    }
}
