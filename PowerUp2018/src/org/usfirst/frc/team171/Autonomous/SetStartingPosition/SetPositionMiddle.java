package org.usfirst.frc.team171.Autonomous.SetStartingPosition;

import org.usfirst.frc.team171.robot.commands.SetRobotPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class SetPositionMiddle extends CommandGroup {

    public SetPositionMiddle() {
    	addSequential(new SetRobotPosition(167.25, 20.375));
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
