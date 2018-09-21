package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PickUpCube;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlatformClearRight;
import org.usfirst.frc.team171.Autonomous.SetStartingPosition.SetPositionRight;
import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.commands.FlippyDowny;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;
import org.usfirst.frc.team171.robot.commands.ResetGyro;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StartFromRight extends CommandGroup {

	String message;

	public StartFromRight() {

		message = DriverStation.getInstance().getGameSpecificMessage();
		
//		message = Robot.getGSM();
		
		addSequential(new JoystickEnabled(false));
		addParallel(new FlippyDowny(1));
		addSequential(new ResetGyro());
		addSequential(new SetPositionRight());
				
		if (message.length() > 0) {
			if (message.charAt(0) == 'R') {
				addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.SIDE));
				addSequential(new PickUpCube(6));

				switch (message.charAt(1)) {
				case 'R':
					addSequential(new PlaceCubeRightScale());
					break;
				case 'L':
					addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
					addSequential(new PickUpCube(2));
					addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
					break;
				}
			} else if (message.charAt(1) == 'R') {
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
		else
		{
			addSequential(new DriveStraightLeft());
		}

	}
}
