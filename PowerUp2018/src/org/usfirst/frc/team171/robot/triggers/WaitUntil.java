package org.usfirst.frc.team171.robot.triggers;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;

public class WaitUntil {

	private double m_fieldX;
	private double m_fieldY;
	private double m_elevatorHeight;
	private double xTolerance = 2;
	private double yTolerance = 2;
	private double elevatorTolerance = 2;

	public WaitUntil(double fieldX, double fieldY, double elevatorHeight) {
		this.m_fieldX = fieldX;
		this.m_fieldY = fieldY;
		this.m_elevatorHeight = elevatorHeight;
	}

	public boolean inTolerance() {
		return Math.abs(Robot.driveTrain.robotPosition()[0] - this.m_fieldX) < xTolerance
				&& Math.abs(Robot.driveTrain.robotPosition()[1] - this.m_fieldY) < yTolerance
				&& Math.abs(RobotMap.elevator.getElevatorPosition() - this.m_elevatorHeight) < elevatorTolerance;
	}
	
	public boolean xInTolerance() {
		return Math.abs(Robot.driveTrain.robotPosition()[0] - this.m_fieldX) < xTolerance;
	}
	
	public boolean yInTolerance() {
		return Math.abs(Robot.driveTrain.robotPosition()[1] - this.m_fieldY) < yTolerance;
	}
	
	public boolean elevatorInTolerance() {
		return Math.abs(RobotMap.elevator.getElevatorPosition() - this.m_elevatorHeight) < elevatorTolerance;
	}
}
