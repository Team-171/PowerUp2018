package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.RobotParts.SwerveModule;
import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.RobotMap;
import org.usfirst.frc.team171.robot.commands.JoystickEnabled;
import org.usfirst.frc.team171.robot.triggers.RunFromJoystick;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public boolean fieldOriented = true;
	public boolean rotating = false;
	public boolean swerveDisabled = false;
	private double robotWidth = 22.5;
	private double robotLength = 22.5;
	private int rate = 50;// Hz
	private SwerveModule m_leftFront;
	private SwerveModule m_leftBack;
	private SwerveModule m_rightFront;
	private SwerveModule m_rightBack;

	private final double[] ws = new double[4];

	public DriveTrain(SwerveModule leftFront, SwerveModule leftBack, SwerveModule rightFront, SwerveModule rightBack) {
		this.m_leftFront = leftFront;
		this.m_leftBack = leftBack;
		this.m_rightFront = rightFront;
		this.m_rightBack = rightBack;

		Thread positionThread = new Thread(() -> positionErrorCorrecting());
		positionThread.setDaemon(true);
		positionThread.setName("Position Error Correcting");
		positionThread.start();
	}

	private void positionErrorCorrecting() {
		while (true) {
			setRobotPosition(robotPosition());

			try {
				Thread.sleep(1000 / rate);
			} catch (InterruptedException ex) {
			}
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new RunFromJoystick());
	}

	/*
	 * @param x x-axis movement, from -1.0 (left) to 1.0 (right)
	 * 
	 * @param y y-axis movement, from -1.0 (reverse) to 1.0 (forward)
	 * 
	 * @param rotation robot rotation, from -1.0 (CCW) to 1.0 (CW)
	 */
	public void driveSwerve(double x, double y, double rotation) {
		boolean angleOptimization = true;
		boolean speedControl = false;

		// if(speedControl)
		// {
		// double speedMultiplier = Robot.oi.gamepad.getRawAxis(2) * 0.2;
		//
		// x *= 0.8 + speedMultiplier;
		// y *= 0.8 + speedMultiplier;
		// }

		if (fieldOriented) {
			final double angle = Robot.gyro.getGyroAngle() * Math.PI / 180.0;
			final double temp = y * Math.cos(angle) + x * Math.sin(angle);
			x = -y * Math.sin(angle) + x * Math.cos(angle);
			y = temp;
		} else {
			// x *= .5;
		}

		if (swerveDisabled && (x != 0 || y != 0 || rotation != 0)) {
			swerveDisabled = false;
			this.m_leftFront.getPID().enable();
			this.m_leftBack.getPID().enable();
			this.m_rightFront.getPID().enable();
			this.m_rightBack.getPID().enable();
		}

		if (!swerveDisabled && x == 0 && y == 0 && rotation == 0) {
			swerveDisabled = true;
			this.m_leftFront.getPID().disable();
			this.m_leftBack.getPID().disable();
			this.m_rightFront.getPID().disable();
			this.m_rightBack.getPID().disable();
		}

		// if (rotating && rotation == 0.0 && Robot.joystickRunning) {
		// rotating = false;
		// Robot.gyro.setTargetAngle(Robot.gyro.getGyroAngle());
		// }
		//
		// if (!rotating || !Robot.joystickRunning) {
		// if (Math.abs(rotation) > 0.0) {
		// rotating = true;
		// } else {
		// if (x != 0 || y != 0) {
		// rotation = Robot.gyro.getTargetYawComp();
		// }
		// }
		// }

		// SmartDashboard.putNumber("x", x);
		// SmartDashboard.putNumber("y", y);
		// SmartDashboard.putNumber("rotation", rotation);

		SmartDashboard.putNumber("Trigger", Robot.oi.gamepad.getRawAxis(2));

		final double LENGTH = 1.0;
		final double WIDTH = 1.0;
		final double RADIUS = Math.hypot(LENGTH, WIDTH);

		final double a = x - rotation * (LENGTH / RADIUS);
		final double b = x + rotation * (LENGTH / RADIUS);
		final double c = y - rotation * (WIDTH / RADIUS);
		final double d = y + rotation * (WIDTH / RADIUS);

		ws[0] = Math.hypot(b, d);
		ws[1] = Math.hypot(a, d);
		ws[2] = Math.hypot(b, c);
		ws[3] = Math.hypot(a, c);

		final double maxSpeed = 1;
		final double maxWheelSpeed = Math.max(Math.max(ws[0], ws[1]), Math.max(ws[2], ws[3]));
		if (maxWheelSpeed > maxSpeed) {
			for (int i = 0; i < 4; i++) {
				ws[i] /= maxWheelSpeed;
			}
		}

		this.m_leftFront.set((Math.atan2(b, d) * 180 / Math.PI), ws[0], angleOptimization);
		this.m_leftBack.set((Math.atan2(a, d) * 180 / Math.PI), ws[1], angleOptimization);
		this.m_rightFront.set((Math.atan2(b, c) * 180 / Math.PI), ws[2], angleOptimization);
		this.m_rightBack.set((Math.atan2(a, c) * 180 / Math.PI), ws[3], angleOptimization);

		// SmartDashboard.putData(Robot.imu.setName(subsystem, name););

		// SmartDashboard.putNumber("error",
		// this.m_leftFront.PIDController.getPIDController().get());
		// SmartDashboard.putNumber("Target Angle", (Math.atan2(b, c) * 180 /
		// Math.PI));
	}

	public void driveSwervePolar(double angle, double magnitude, double rotation) {
		driveSwerve(Math.cos(angle * (Math.PI / 180)) * magnitude, Math.sin(angle * (Math.PI / 180)) * magnitude,
				rotation);
	}

	public void stopSwerve() {
		driveSwerve(0, 0, 0);
	}

	public double[] robotPosition() {
		return new double[] {
				(this.m_leftFront.fieldX + this.m_leftBack.fieldX + this.m_rightFront.fieldX + this.m_rightBack.fieldX)
						/ 4,
				(this.m_leftFront.fieldY + this.m_leftBack.fieldY + this.m_rightFront.fieldY + this.m_rightBack.fieldY)
						/ 4 };

	}

	public void setRobotPosition(double[] fieldXY) {
		this.m_leftFront.setFieldXY(
				(robotWidth * Math.cos(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 45))) + fieldXY[0],
				(robotLength * Math.sin(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 45))) + fieldXY[1]);
		this.m_leftBack.setFieldXY(
				(robotWidth * Math.cos(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 135))) + fieldXY[0],
				(robotLength * Math.sin(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 135))) + fieldXY[1]);
		this.m_rightFront.setFieldXY(
				(robotWidth * Math.cos(Math.toRadians(Robot.gyro.getUnitCircleAngle() - 45))) + fieldXY[0],
				(robotLength * Math.sin(Math.toRadians(Robot.gyro.getUnitCircleAngle() - 45))) + fieldXY[1]);
		this.m_rightBack.setFieldXY(
				(robotWidth * Math.cos(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 225))) + fieldXY[0],
				(robotLength * Math.sin(Math.toRadians(Robot.gyro.getUnitCircleAngle() + 225))) + fieldXY[1]);
	}

	public boolean isMoving() {
		return Math.abs(this.m_leftFront.getDriveMotor().getMotorOutputPercent()) > 0
				|| Math.abs(this.m_leftBack.getDriveMotor().getMotorOutputPercent()) > 0
				|| Math.abs(this.m_rightFront.getDriveMotor().getMotorOutputPercent()) > 0
				|| Math.abs(this.m_rightBack.getDriveMotor().getMotorOutputPercent()) > 0;
	}

	public void updateStatus() {
		// SmartDashboard.putString("Front Left Position", "X: " +
		// this.m_leftFront.fieldX + ", Y: " +
		// this.m_leftFront.fieldY);
		// SmartDashboard.putString("Back Left Position", "X: " +
		// this.m_leftBack.fieldX + ", Y: " +
		// this.m_leftBack.fieldY);
		//
		// SmartDashboard.putString("Front Right Position", "X: " +
		// this.m_rightFront.fieldX + ", Y: " +
		// this.m_rightFront.fieldY);
		// SmartDashboard.putString("Back Right Position", "X: " +
		// this.m_rightBack.fieldX + ", Y: " +
		// this.m_rightBack.fieldY);
		//
		// Robot.table.putNumber("Front Left Position X",
		// this.m_leftFront.fieldX);
		// Robot.table.putNumber("Front Left Position Y",
		// this.m_leftFront.fieldY);
		//
		// Robot.table.putNumber("Back Left Position X",
		// this.m_leftBack.fieldX);
		// Robot.table.putNumber("Back Left Position Y",
		// this.m_leftBack.fieldY);
		//
		// Robot.table.putNumber("Front Right Position X",
		// this.m_rightFront.fieldX);
		// Robot.table.putNumber("Front Right Position Y",
		// this.m_rightFront.fieldY);
		//
		// Robot.table.putNumber("Back Right Position X",
		// this.m_rightBack.fieldX);
		// Robot.table.putNumber("Back Right Position Y",
		// this.m_rightBack.fieldY);

		SmartDashboard.putNumber("Robot Position X", robotPosition()[0]);
		SmartDashboard.putNumber("Robot Position Y", robotPosition()[1]);

		// SmartDashboard.putNumber("Front Left Error",
		// this.m_leftFront.PIDController.getError());
		// SmartDashboard.putNumber("Back Left Error",
		// this.m_leftBack.PIDController.getError());
		// SmartDashboard.putNumber("Front Right Error",
		// this.m_rightFront.PIDController.getError());
		// SmartDashboard.putNumber("Back Right Error",
		// this.m_rightBack.PIDController.getError());

		// SmartDashboard.putNumber("Encoder",
		// this.m_leftFront.driveEncoder.get());

		// SmartDashboard.putNumber("Front Left Test Angle",
		// this.m_leftFront.testAng);
		// SmartDashboard.putNumber("Back Left Test Angle",
		// this.m_leftBack.testAng);
		// SmartDashboard.putNumber("Front Right Test Angle",
		// this.m_rightFront.testAng);
		// SmartDashboard.putNumber("Back Right Test Angle",
		// this.m_rightBack.testAng);
	}
}
