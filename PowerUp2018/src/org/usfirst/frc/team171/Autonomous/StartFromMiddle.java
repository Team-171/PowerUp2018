package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PickUpCube;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightSwitch;
import org.usfirst.frc.team171.Autonomous.SetStartingPosition.SetPositionMiddle;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StartFromMiddle extends CommandGroup {

	String message;

	public StartFromMiddle() {

		addParallel(new JoystickEnabled(false));
		addParallel(new SetPositionMiddle());
		message = DriverStation.getInstance().getGameSpecificMessage();

		if (message.length() > 0) {
			// This first if statement goes to whichever side of the switch is
			// ours and dumps the cube
			if (message.substring(0, 0) == "L") {
				addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.SIDE));
				addSequential(new PickUpCube(1));

				// Check to see if scale is on our side. If not, we pile cubes
				// on the switch
				if (message.substring(1, 1) == "L") {
					addSequential(new PlaceCubeLeftScale());
					addSequential(new PickUpCube(2));
					addSequential(new PlaceCubeLeftScale());
				} else {
					addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.BACK));
					addSequential(new PickUpCube(2));
					addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.BACK));
				}
			} else {
				addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.SIDE));
				addSequential(new PickUpCube(6));

				// Check to see if scale is on our side. If not, we pile cubes
				// on the switch
				if (message.substring(1, 1) == "R") {
					addSequential(new PlaceCubeRightScale());
					addSequential(new PickUpCube(5));
					addSequential(new PlaceCubeRightScale());
				} else {
					addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
					addSequential(new PickUpCube(5));
					addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
				}
			}
		}

	}
}
