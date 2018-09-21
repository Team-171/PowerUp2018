package org.usfirst.frc.team171.Autonomous.Actions;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.PIDsubsystems.AutoMovementX;
import org.usfirst.frc.team171.robot.PIDsubsystems.AutoMovementY;
import org.usfirst.frc.team171.robot.subsystems.Gyro;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class WayPoint extends Command {

	public static AutoMovementX PIDX = new AutoMovementX();
	public static AutoMovementY PIDY = new AutoMovementY();
	private double m_targetX;
	private double m_targetY;
	private double m_targetAngle;
	private double m_speed;
	private double initialAngle;
	private double initialDistance;
	private double[] lastPosition;
	private double[] targetPosition;

	public WayPoint(double targetX, double targetY, double targetAngle, double speed) {
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
		Robot.gyro.gyroKp = 0.01;
		lastPosition = Robot.driveTrain.robotPosition();
		this.targetPosition = new double[] { m_targetX, m_targetY };
		Robot.driveTrain.driveSwerve(WayPoint.PIDX.getPIDController().get(), WayPoint.PIDY.getPIDController().get(), 0);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if(Math.abs(angleTo(lastPosition, Robot.driveTrain.robotPosition()) - angleTo(Robot.driveTrain.robotPosition(), targetPosition)) > 10)
		{
			Robot.driveTrain.driveSwerve(WayPoint.PIDX.getPIDController().get(), WayPoint.PIDY.getPIDController().get(), 0);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return distanceTo(Robot.driveTrain.robotPosition(), targetPosition) < 3;
	}

	// Called once after isFinished returns true
	protected void end() {
		WayPoint.PIDX.disable();
		WayPoint.PIDY.disable();
		Robot.driveTrain.stopSwerve();
		Robot.gyro.gyroKp = 0.004;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	private double angleTo(double[] initial, double[] current) {
		double xDist = xDist(initial, current);
		double yDist = yDist(initial, current);
		double angleRad = Math.atan2(yDist, xDist);
		double angleDeg = Math.toDegrees(angleRad);

		return Gyro.toGyroCircleAngle(angleDeg);
	}

	private double distanceTo(double[] initial, double[] current){
		return Math.hypot(xDist(initial, current), yDist(initial, current));
	}
	
	private double xDist(double[] initial, double[] current){
		return current[0] - initial[0];
	}
	
	private double yDist(double[] initial, double[] current){
		return current[1] - initial[1];
	}

	public double getTargetX() {
		return this.m_targetX;
	}

	public double getTargetY() {
		return this.m_targetY;
	}

	public double getSpeed() {
		return this.m_speed;
	}
}
