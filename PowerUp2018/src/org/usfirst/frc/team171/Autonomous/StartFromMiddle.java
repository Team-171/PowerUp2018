package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PickUpCube;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightSwitch;
import org.usfirst.frc.team171.Autonomous.SetStartingPosition.SetPositionMiddle;
import org.usfirst.frc.team171.robot.commands.FlippyDowny;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;
import org.usfirst.frc.team171.robot.commands.ResetGyro;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StartFromMiddle extends CommandGroup {

	String message;

	public StartFromMiddle() {
		
		message = DriverStation.getInstance().getGameSpecificMessage();

		addParallel(new JoystickEnabled(false));
		addParallel(new FlippyDowny(1));
		addSequential(new ResetGyro());
		addSequential(new SetPositionMiddle());

		if (message.length() > 0) {
			// This first if statement goes to whichever side of the switch is
			// ours and dumps the cube
			if (message.charAt(0) == 'L') {
				addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.FRONT));
//				addSequential(new PickUpCube(1));

				// Check to see if scale is on our side. If not, we pile cubes
				// on the switch
//				if (message.charAt(1) == 'L') {
//					addSequential(new PlaceCubeLeftScale());
//					addSequential(new PickUpCube(2));
//					addSequential(new PlaceCubeLeftScale());
//				} else {
//					addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.BACK));
//					addSequential(new PickUpCube(2));
//					addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.BACK));
//				}
			} else {
				addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.FRONT));
//				addSequential(new PickUpCube(6));

				// Check to see if scale is on our side. If not, we pile cubes
				// on the switch
//				if (message.charAt(1) == 'R') {
//					addSequential(new PlaceCubeRightScale());
//					addSequential(new PickUpCube(5));
//					addSequential(new PlaceCubeRightScale());
//				} else {
//					addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
//					addSequential(new PickUpCube(5));
//					addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
//				}
			}
		}

	}
}
