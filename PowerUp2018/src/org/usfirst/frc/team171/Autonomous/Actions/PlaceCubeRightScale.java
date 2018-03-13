package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.Autonomous.WayPoint;
import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class PlaceCubeRightScale extends CommandGroup {

    public PlaceCubeRightScale() {
    	addSequential(new PlatformClearLeft(315));
    	addParallel(new WayPoint(259, 280, 335, 0.5));
    	addParallel(new SetElevatorPosition(10)); // TODO: Set actual position
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
