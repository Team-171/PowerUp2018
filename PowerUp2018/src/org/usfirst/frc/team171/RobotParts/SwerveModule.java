package org.usfirst.frc.team171.RobotParts;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.PIDsubsystems.PositionWheel;

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
	public double fieldX = 0.0;
	public double fieldY = 0.0;
	public double forwardAngle = 0;
	
	private boolean reversed = false;
	private int rate = 200; // Hz
	private double lastAngle;
	private double lastEncoderReading;
	private double wheelSize = 5.3770491803278688524590163934426;//4.0;// wheel size in inches
	private double calculatedEncoderCount = 0.0;
	private double lastCalculatedEncoderCount = 0.0;
	private double wheelToEncoderRatio = 3.2;
	private double moduleRotationRatio = 2.3;
	private double countsPerRev = 400;
	public double testAng = 0;

	public SwerveModule(PWMTalonSRX drive, Encoder driveEnc, PWMTalonSRX dir, AbsoluteEncoder dirEnc, double forwardAng) {
		this.driveMotor = drive;
		this.driveEncoder = driveEnc;
		this.directionMotor = dir;
		this.directionEncoder = dirEnc;
		this.PIDController = new PositionWheel(this);
		this.forwardAngle = forwardAng;

		lastAngle = directionEncoder.getAngle();
		lastEncoderReading = lastCalculatedEncoderCount = driveEncoder.get();

		Thread positionThread = new Thread(() -> calculatePosition());
		positionThread.setDaemon(true);
		positionThread.setName("Custom Super Fast Loop");
		positionThread.start();
//		SmartDashboard.putData("SwervePID", PIDController.getPIDController());
	}

	private void calculatePosition() {
		while (true) {
			calculatedEncoderCount += driveEncoder.get() - lastEncoderReading;
			
			double currentAngle = directionEncoder.getAngle() - forwardAngle;
			
			if(currentAngle > 360)
			{
				currentAngle -= 360;
			}
			
			if(currentAngle<0)
			{
				currentAngle += 360;
			}
			
			if(Math.abs(currentAngle - lastAngle)>1)
			{
				double angleDifference;
				
				if(currentAngle<10 && lastAngle > 350)
				{
					angleDifference = (360 - lastAngle) + currentAngle;
				}
				else if(lastAngle<10 && currentAngle > 350)
				{
					angleDifference = (currentAngle - 360) - lastAngle;
				}
				else
				{
					angleDifference = currentAngle  - lastAngle;
				}
				
				calculatedEncoderCount += (angleDifference / 360) * moduleRotationRatio * countsPerRev;
			}
			
			double fieldAngle = currentAngle + Robot.gyro.getGyroAngle();
			
			if(fieldAngle < 0)
			{
				fieldAngle += 360;
			}
			if(fieldAngle > 360)
			{
				fieldAngle -= 360;
			}
			
			fieldX += Math.cos(Math.toRadians(Robot.gyro.toUnitCircleAngle(fieldAngle))) * (((calculatedEncoderCount - lastCalculatedEncoderCount) / countsPerRev / wheelToEncoderRatio) * (Math.PI * wheelSize));// (calculatedEncoderCount - lastCalculatedEncoderCount);
			fieldY += Math.sin(Math.toRadians(Robot.gyro.toUnitCircleAngle(fieldAngle))) * (((calculatedEncoderCount - lastCalculatedEncoderCount) / countsPerRev / wheelToEncoderRatio) * (Math.PI * wheelSize));
			

			lastAngle = currentAngle;
			lastEncoderReading = driveEncoder.get();
			lastCalculatedEncoderCount = calculatedEncoderCount;

			try {
				Thread.sleep(1000 / rate);
			} catch (InterruptedException ex) {
			}
		}
	}
	
	public void setReversed(boolean m_reversed){
		this.reversed = m_reversed;
	}

	public void set(double m_angle, double m_speed, boolean angleOptimization) {
		setAngle(m_angle, angleOptimization);
		setSpeed(m_speed);
//		SmartDashboard.putData("SwervePID", PIDController.getPIDController());
	}
	
	public void setForwardAngle(double m_Angle){
		this.forwardAngle = m_Angle;
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

	public void setAngle(double targetAngle, boolean angleOptimization) {
		targetAngle += forwardAngle;
		
		if(targetAngle > 360)
		{
			targetAngle -= 360;
		}
		
		if(targetAngle<0)
		{
			targetAngle += 360;
		}

		if(angleOptimization)
		{
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
		}
		
		angle = targetAngle;
		PIDController.setDesiredAngle(targetAngle);

	}
	
	public void setFieldXY(double m_fieldX, double m_fieldY){
		this.fieldX = m_fieldX;
		this.fieldY = m_fieldY;
	}
}
