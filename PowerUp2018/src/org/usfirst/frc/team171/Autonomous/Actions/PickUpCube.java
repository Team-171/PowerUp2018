package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;
import org.usfirst.frc.team171.RobotMotion.SetIntakeSpeed;
import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PickUpCube extends CommandGroup {

    public PickUpCube(int cubeNumber) {
    	
    	switch (cubeNumber) {
    	case 1:
    		addSequential(new PlatformClearRight(225));
    		addSequential(new WayPoint(233, 235, 180, 0.5));
    		addSequential(new SetIntakeSpeed(0.5));
    		addSequential(new WayPoint(233, 225, 180, 0.25));
    		break;
    		
    	case 2:
    		addSequential(new PlatformClearRight(225));
    		addSequential(new WayPoint(204, 235, 180, 0.5));
    		addSequential(new SetIntakeSpeed(0.5));
    		addSequential(new WayPoint(204, 225, 180, 0.25));
    		break;
    		
    	case 3:
 
    		break;
    	case 4:
    		
    		break;
    	case 5:
    		addSequential(new PlatformClearLeft(135));
    		addSequential(new WayPoint(120, 235, 180, 0.5));
    		addSequential(new SetIntakeSpeed(0.5));
    		addSequential(new WayPoint(120, 225, 180, 0.25));
    		break;
    	case 6:
    		addSequential(new PlatformClearLeft(135));
    		addSequential(new WayPoint(92, 235, 180, 0.5));
    		addSequential(new SetIntakeSpeed(0.5));
    		addSequential(new WayPoint(92, 225, 180, 0.25));
    		break;
    	}
    }
}
