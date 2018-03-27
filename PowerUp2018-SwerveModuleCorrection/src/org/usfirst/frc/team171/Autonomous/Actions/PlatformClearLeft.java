package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PlatformClearLeft extends CommandGroup {
    public PlatformClearLeft(double angle) {
    	addSequential(new WayPoint(65, 235, angle, 0.5));
    }
}
