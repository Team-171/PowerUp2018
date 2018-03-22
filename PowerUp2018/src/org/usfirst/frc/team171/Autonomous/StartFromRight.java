package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PickUpCube;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlatformClearRight;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StartFromRight extends CommandGroup {
	
	String message;

    public StartFromRight() {
    	
    	addParallel(new JoystickEnabled(false));
    	message = DriverStation.getInstance().getGameSpecificMessage();
    	
    	if (message.substring(0, 0) == "R") {
    		addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.SIDE));
    		addSequential(new PickUpCube(6));
    		
    		switch (message.substring(1, 1)) {
        	case "R":
        		addSequential(new PlaceCubeRightScale());
        		break;
        	case "L":    		
        		addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
        		addSequential(new PickUpCube(2));
        		addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
        		break;
    		}
    	} else if (message.substring(1, 1) == "R") {
    		addSequential(new PlaceCubeRightScale());
    		addSequential(new PickUpCube(6));
    		addSequential(new PlaceCubeRightScale());
    	} else {
    		addSequential(new PlatformClearRight(270));
    		addSequential(new PlaceCubeLeftScale());
    		addSequential(new PickUpCube(1));
    		addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.BACK));
    		addSequential(new PickUpCube(2));
    		addSequential(new PlaceCubeLeftScale());
    	}
    }
}
