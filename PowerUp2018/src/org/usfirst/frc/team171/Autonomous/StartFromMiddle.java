package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightSwitch;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StartFromMiddle extends CommandGroup {

    public StartFromMiddle() {
    	addParallel(new JoystickEnabled(false));
    	
    	if (DriverStation.getInstance().getGameSpecificMessage().substring(1, 1) == "L") {
    		addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.SIDE));    	
    	} else {
    		addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.SIDE));
    	}
    }
}
