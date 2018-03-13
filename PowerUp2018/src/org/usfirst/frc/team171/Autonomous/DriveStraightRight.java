package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PickUpCube;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveStraightRight extends CommandGroup {

    public DriveStraightRight() {
addParallel(new JoystickEnabled(false));
    	
    	if (DriverStation.getInstance().getGameSpecificMessage().substring(0, 0) == "R") {
    		addSequential(new PlaceCubeLeftSwitch(PlaceCubeRightSwitch.Direction.SIDE));
    		addSequential(new PickUpCube(6));
    	}

    	switch (DriverStation.getInstance().getGameSpecificMessage().substring(1, 1)) {
    	case "L":
    		addSequential(new PlaceCubeLeftScale());
    		break;
    	case "R":    		
    		addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.BACK));
    		break;
    	}
    }
}
