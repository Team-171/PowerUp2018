package org.usfirst.frc.team171.Autonomous.SetStartingPosition;

import org.usfirst.frc.team171.robot.commands.SetRobotPosition;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetPositionMiddlee extends InstantCommand {

    public SetPositionMiddlee() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	new SetRobotPosition(167.25, 20.375).start();
    }

}
