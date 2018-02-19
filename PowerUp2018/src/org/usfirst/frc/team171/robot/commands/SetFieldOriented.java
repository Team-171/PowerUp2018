package org.usfirst.frc.team171.robot.commands;

import org.usfirst.frc.team171.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetFieldOriented extends InstantCommand {
	private boolean m_fieldOriented;

    public SetFieldOriented(boolean fieldOriented) {
        super();
        this.m_fieldOriented = fieldOriented;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called once when the command executes
    protected void initialize() {
    	Robot.driveTrain.fieldOriented = m_fieldOriented;
    }

}
