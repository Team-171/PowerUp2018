package org.usfirst.frc.team171.RobotMotion;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.subsystems.AbsoluteEncoder;
import org.usfirst.frc.team171.robot.subsystems.PositionWheel;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

	public PWMTalonSRX driveMotor;
	public Encoder driveEncoder;
	public PWMTalonSRX directionMotor;
	public AbsoluteEncoder directionEncoder;
	public double speed;
	public double angle;
	public PositionWheel PIDController;
	private boolean reversed = false;
	private int rate = 200; // Hz
	private double lastAngle;
	private double lastEncoderReading;
	private double wheelSize = 4.0;// wheel size in inches
	private double calculatedEncoderCount = 0.0;
	private double lastCalculatedEncoderCount;
	private double wheelToEncoderRatio = 3.2;
	private double moduleRotationRatio = 2.3;
	private double countsPerRev = 400;
	public double fieldX = 0.0;
	public double fieldY = 0.0;

	public SwerveModule(PWMTalonSRX drive, Encoder driveEnc, PWMTalonSRX dir, AbsoluteEncoder dirEnc) {
		this.driveMotor = drive;
		this.driveEncoder = driveEnc;
		this.directionMotor = dir;
		this.directionEncoder = dirEnc;
		this.PIDController = new PositionWheel(this);

		lastAngle = directionEncoder.getAngle();
		lastEncoderReading = lastCalculatedEncoderCount = driveEncoder.get();

		Thread positionThread = new Thread(() -> calculatePosition());
		positionThread.setDaemon(true);
		positionThread.setName("Custom Super Fast Loop");
		positionThread.start();
		SmartDashboard.putData("SwervePID", PIDController.getPIDController());
	}

	private void calculatePosition() {
		while (true) {
			calculatedEncoderCount += driveEncoder.get() - lastEncoderReading;
			
			if(directionEncoder.getAngle() != lastAngle)
			{
				double angleDifference;
				
				if(directionEncoder.getAngle()<10 && lastAngle > 350)
				{
					angleDifference = (360 - lastAngle) + directionEncoder.getAngle();
				}
				else if(lastAngle<10 && directionEncoder.getAngle() > 350)
				{
					angleDifference = (directionEncoder.getAngle() - 360) - lastAngle;
				}
				else
				{
					angleDifference = directionEncoder.getAngle() - lastAngle;
				}
				
				calculatedEncoderCount += (angleDifference / 360) * moduleRotationRatio * countsPerRev;
			}
			
			double fieldAngle = directionEncoder.getAngle() + Robot.gyro.getGyroAngle();
			
			if(fieldAngle < 0)
			{
				fieldAngle += 360;
			}
			if(fieldAngle > 360)
			{
				fieldAngle -= 360;
			}
			
			fieldX += Math.cos(fieldAngle) * (calculatedEncoderCount - lastCalculatedEncoderCount);
			fieldY += Math.sin(fieldAngle) * (calculatedEncoderCount - lastCalculatedEncoderCount);
			

			lastAngle = directionEncoder.getAngle();
			lastEncoderReading = driveEncoder.get();
			lastCalculatedEncoderCount = calculatedEncoderCount;

			try {
				Thread.sleep(1000 / rate);
			} catch (InterruptedException ex) {
			}
		}
	}

	public void set(double m_angle, double m_speed) {
		setAngle(m_angle);
		setSpeed(m_speed);
		SmartDashboard.putData("SwervePID", PIDController.getPIDController());
	}

	public void setSpeed(double targetSpeed) {
		if (targetSpeed > 1) {
			targetSpeed = 1;
		}

		if (targetSpeed < -1) {
			targetSpeed = -1;
		}

		if (reversed) {
			targetSpeed = -targetSpeed;
		}

		speed = targetSpeed;
		driveMotor.set(targetSpeed);
	}

	public void setAngle(double targetAngle) {
		targetAngle = (360 - targetAngle);

		if (Math.abs(targetAngle - directionEncoder.getAngle()) > 90
				&& Math.abs(360 - targetAngle + directionEncoder.getAngle()) > 90) {
			reversed = true;
			if (targetAngle < 180) {
				targetAngle += 180;
			} else {
				targetAngle -= 180;
			}
		} else {
			reversed = false;
		}
		angle = targetAngle;
		PIDController.setDesiredAngle(targetAngle);

	}
}
