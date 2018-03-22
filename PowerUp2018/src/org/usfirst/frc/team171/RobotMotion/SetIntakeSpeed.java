package org.usfirst.frc.team171.RobotMotion;

import org.usfirst.frc.team171.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class SetIntakeSpeed extends InstantCommand {
	
	double m_speed;
	
    public SetIntakeSpeed(double speed) {
        super();
        this.m_speed = speed;
    }

    // Called once when the command executes
    protected void initialize() {
    	RobotMap.intake.runIntake(this.m_speed, this.m_speed);
    }

}
