package org.usfirst.frc.team171.Autonomous;


import org.usfirst.frc.team171.Autonomous.SetStartingPosition.SetPositionLeft;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveStraightLeftt extends CommandGroup {

    public DriveStraightLeftt() {
    	addParallel(new JoystickEnabled(false));
    	addSequential(new SetPositionLeft());
    	addSequential(new WayPoint(45.5, 167.5, 90, 0.6), 5);
    }
}
