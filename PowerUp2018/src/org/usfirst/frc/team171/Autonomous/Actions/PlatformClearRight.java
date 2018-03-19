package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PlatformClearRight extends CommandGroup {
    public PlatformClearRight(double angle) {
    	addSequential(new WayPoint(259, 235, angle, 0.5));
    }
}
