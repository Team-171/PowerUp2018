package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.RobotMotion.SetIntakeSpeed;
import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.commands.SetIntake;
import org.usfirst.frc.team171.robot.commands.TimeDelay;
import org.usfirst.frc.team171.robot.subsystems.JeVois;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PickUpCube extends CommandGroup {

	JeVois jeVois;

	public PickUpCube(int cubeNumber) {

		// jeVois = new JeVois();

		switch (cubeNumber) {
		case 1:
			addSequential(new PlatformClearLeft(135));
			addSequential(new WayPoint(92, 235, 180, 0.6));
			addSequential(new SetIntakeSpeed(0.5));
			addSequential(new WayPoint(92, 225, 180, 0.25));
			addParallel(new SetIntake(null, 0.75));
			addSequential(new TimeDelay(0.5));
			addSequential(new SetIntakeSpeed(0));
			addSequential(new WayPoint(92, 235, 180, 0.6));
			break;
		case 2:
			addSequential(new PlatformClearLeft(135));
			addSequential(new WayPoint(120, 235, 180, 0.5));
			addSequential(new SetIntakeSpeed(0.5));
			addSequential(new WayPoint(120, 225, 180, 0.25));
			addParallel(new SetIntake(null, 0.75));
			addSequential(new TimeDelay(0.5));
			addSequential(new SetIntakeSpeed(0));
			addSequential(new WayPoint(120, 235, 180, 0.5));
			break;

		case 3:

			break;

		case 4:

			break;

		case 5:
			addSequential(new PlatformClearRight(225));
			addSequential(new WayPoint(204, 235, 180, 0.5));
			addSequential(new SetIntakeSpeed(0.5));
			addSequential(new WayPoint(204, 225, 180, 0.25));
			addParallel(new SetIntake(null, 0.75));
			addSequential(new TimeDelay(0.5));
			addSequential(new SetIntakeSpeed(0));
			addSequential(new WayPoint(204, 235, 180, 0.5));
			break;
		case 6:
			addSequential(new PlatformClearRight(225));
			addSequential(new WayPoint(233, 235, 180, 0.5));
			addParallel(new SetIntakeSpeed(0.5));
			addSequential(new WayPoint(233, 225, 180, 0.25));
			addParallel(new SetIntake(null, 0.75));
			addSequential(new TimeDelay(0.5));
			addSequential(new SetIntakeSpeed(0));
			addSequential(new WayPoint(233, 235, 180, 0.5));
			break;
		}
	}

	// TODO: Find out placement of camera and finish function
	protected void centerOnCube() {
		String[] tmp = jeVois.read().split(" ");
		int[] cubeCoordinates = new int[tmp.length];
		int resWidth = 640;
		int resHeight = 480;
		int cubeCenterX;
		int cubeCenterY;

		for (int i = 0; i < tmp.length - 1; i++) {
			cubeCoordinates[i] = Integer.parseInt(tmp[i]);
		}

		cubeCenterX = (cubeCoordinates[0] + cubeCoordinates[2]) / 2;
		cubeCenterY = (cubeCoordinates[1] + cubeCoordinates[3]) / 2;

		if (Math.abs(resWidth - cubeCenterX) > 5) {

		}

	}
}
