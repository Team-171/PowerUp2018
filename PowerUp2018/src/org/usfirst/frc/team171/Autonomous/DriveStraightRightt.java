package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.Autonomous.Actions.PickUpCube;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftScale;
import org.usfirst.frc.team171.Autonomous.Actions.PlaceCubeLeftSwitch;
import org.usfirst.frc.team171.Autonomous.Actions.WayPoint;
import org.usfirst.frc.team171.Autonomous.SetStartingPosition.SetPositionRight;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveStraightRightt extends CommandGroup {

    public DriveStraightRightt() {
    	addParallel(new JoystickEnabled(false));
    	addSequential(new SetPositionRight());
    	addSequential(new WayPoint(278.5, 280, 335, 0.5), 6);
    }
}
