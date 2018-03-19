package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PickUpCube;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightSwitch;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StartFromRight extends CommandGroup {

    public StartFromRight() {
    	
    	addParallel(new JoystickEnabled(false));
    	
    	if (DriverStation.getInstance().getGameSpecificMessage().substring(0, 0) == "R") {
    		addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.SIDE));
    		addSequential(new PickUpCube(1));
    	}

    	switch (DriverStation.getInstance().getGameSpecificMessage().substring(1, 1)) {
    	case "R":
    		addSequential(new PlaceCubeRightScale());
    		break;
    	case "L":    		
    		addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
    		addSequential(new PickUpCube(5));
    		addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
    		break;
    	}
    }
}
