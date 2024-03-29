package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PickUpCube;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeRightSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.PlatformClearLeft;
import org.usfirst.frc.team171.Autonomous.SetStartingPosition.SetPositionLeft;
import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;
import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.commands.FlippyDowny;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;
import org.usfirst.frc.team171.robot.commands.ResetGyro;
import org.usfirst.frc.team171.robot.commands.SetRobotPosition;
import org.usfirst.frc.team171.robot.commands.TimeDelay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class StartFromLeft extends CommandGroup {

	String message;

	public StartFromLeft() {

		message = DriverStation.getInstance().getGameSpecificMessage();// Robot.getGSM();

		addParallel(new JoystickEnabled(false));
		addParallel(new FlippyDowny(1));
		addSequential(new ResetGyro());
		addSequential(new SetPositionLeft());
//		addSequential(new WayPoint(30, 144, 0, 0.5));

		if (message.length() > 0) {
			if (message.charAt(0) == 'L') {
				System.out.println("THING");
				addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.SIDE));
//				addSequential(new TimeDelay(0.25));
				addSequential(new PickUpCube(1));

				switch (message.charAt(1)) {
				case 'L':
					addSequential(new PlaceCubeLeftScale());
					addSequential(new PickUpCube(2));
					addSequential(new PlaceCubeLeftScale());
					break;
				case 'R':
					addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.BACK));
					addSequential(new PickUpCube(2));
					addSequential(new PlaceCubeLeftSwitch(PlaceCubeLeftSwitch.Direction.BACK));
					break;
				}
			} else if (message.charAt(1) == 'L') {
				addSequential(new PlaceCubeLeftScale());
				addSequential(new PickUpCube(1));
				addSequential(new PlaceCubeLeftScale());
			} else {
				addSequential(new PlatformClearLeft(90));
				addSequential(new PlaceCubeRightScale());
				addSequential(new PickUpCube(6));
				addSequential(new PlaceCubeRightSwitch(PlaceCubeRightSwitch.Direction.BACK));
				addSequential(new PickUpCube(5));
				addSequential(new PlaceCubeRightScale());
			}
		}
		else
		{
			addSequential(new DriveStraightLeft());
		}

	}
}
