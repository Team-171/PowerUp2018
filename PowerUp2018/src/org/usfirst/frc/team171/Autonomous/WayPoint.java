package org.usfirst.frc.team171.Autonomous;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.PIDsubsystems.AutoMovementX;
import org.usfirst.frc.team171.robot.PIDsubsystems.AutoMovementY;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class WayPoint extends Command {

	public static AutoMovementX PIDX = new AutoMovementX();
	public static AutoMovementY PIDY = new AutoMovementY();
	private double m_targetX;
	private double m_targetY;
	private double m_targetAngle;
	private double waypointSpacing = 4;
	private double inPositionDistance = 15;
	private double lastInPositionDistance = 4;
	private double initialAngle;
	private double initialDistance;
	private double m_speed;

	private SubWaypoint[] plotPath;
	private int numWaypoints;
	private int currentWaypoint = 0;

	public WayPoint(double targetX, double targetY, double targetAngle, double speed) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);

		this.m_targetX = targetX;
		this.m_targetY = targetY;
		this.m_targetAngle = targetAngle;
		this.m_speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		this.initialAngle = Robot.gyro.getGyroAngle();
		double xDistance = m_targetX - Robot.driveTrain.robotPosition()[0];
		double yDistance = m_targetY - Robot.driveTrain.robotPosition()[1];
		this.initialDistance = Math.hypot(xDistance, yDistance);
		WayPoint.PIDX.enable();
		WayPoint.PIDY.enable();

		numWaypoints = (int) Math.ceil(initialDistance / waypointSpacing);
		SmartDashboard.putNumber("NumWaypoints", numWaypoints);
		double angleToDestination = Robot.gyro.getAngleError(m_targetAngle);
		// SmartDashboard.putNumber("Angle Error", angleToDestination);
		double initialLocation[] = Robot.driveTrain.robotPosition();

		plotPath = new SubWaypoint[numWaypoints];

		for (int i = 0; i < (numWaypoints - 1); i++) {
			double xAddition = (xDistance / initialDistance) * ((i + 1) * waypointSpacing);
			double yAddition = (yDistance / initialDistance) * ((i + 1) * waypointSpacing);
			double angleAddition = angleToDestination * ((i + 1) / (double) numWaypoints);

			// SmartDashboard.putNumber("y" + i, yAddition);

			plotPath[i] = new SubWaypoint((initialLocation[0] + xAddition), (initialLocation[1] + yAddition),
					org.usfirst.frc.team171.robot.subsystems.Gyro.normalizeAngle(initialAngle + angleAddition), m_speed,
					inPositionDistance);

			SmartDashboard.putNumber("SubWaypoint" + i + "X", plotPath[i].getTargetX());
			SmartDashboard.putNumber("SubWaypoint" + i + "Y", plotPath[i].getTargetY());
		}
		plotPath[numWaypoints - 1] = new SubWaypoint(m_targetX, m_targetY, m_targetAngle, m_speed,
				lastInPositionDistance);

		SmartDashboard.putNumber("SubWaypoint" + (numWaypoints - 1) + "X", plotPath[(numWaypoints - 1)].getTargetX());
		SmartDashboard.putNumber("SubWaypoint" + (numWaypoints - 1) + "Y", plotPath[(numWaypoints - 1)].getTargetY());

//		plotPath[0].start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		SmartDashboard.putBoolean("Waypoint Running", plotPath[0].isFinished());

		try {
			if (plotPath[currentWaypoint].isFinished()) {
				plotPath[currentWaypoint].end();
				if (currentWaypoint < (numWaypoints - 1)) {
					plotPath[++currentWaypoint].start();
				}
			}

			plotPath[currentWaypoint].run();
		} catch (Exception e) {
			System.out.println(e.toString() + " Current: " + currentWaypoint + " Numwaypoints: " + numWaypoints);
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return currentWaypoint == (numWaypoints - 1) && plotPath[numWaypoints - 1].isFinished();
	}

	// Called once after isFinished returns true
	protected void end() {
		WayPoint.PIDX.disable();
		WayPoint.PIDY.disable();
		Robot.driveTrain.stopSwerve();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
