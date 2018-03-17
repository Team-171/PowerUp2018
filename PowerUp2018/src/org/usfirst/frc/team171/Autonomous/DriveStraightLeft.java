package org.usfirst.frc.team171.Autonomous;


import org.usfirst.frc.team171.robot.commands.JoystickEnabled;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveStraightLeft extends CommandGroup {

    public DriveStraightLeft() {
    	addParallel(new JoystickEnabled(false));
    	
    }
}
