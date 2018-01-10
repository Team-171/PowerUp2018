package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.robot.commands.PositionWheels;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	double 

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new PositionWheels());
    }
}

