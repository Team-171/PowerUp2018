package org.usfirst.frc.team171.RobotParts;

import org.usfirst.frc.team171.robot.Robot;
import org.usfirst.frc.team171.robot.PIDsubsystems.PositionWheel;
import org.usfirst.frc.team171.robot.subsystems.Gyro;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

	public TalonSRX driveMotor;
	public Encoder driveEncoder;
	public PWMTalonSRX directionMotor;
	public AbsoluteEncoder directionEncoder;
	public double speed;
	public double angle;
	public PositionWheel PIDController;
	public double fieldX = 0.0;
	public double fieldY = 0.0;
	public double forwardAngle = 0;
	public double testAng = 0;
	
	private boolean reversed = false;
	private int rate = 300; // Hz
	private double lastAngle;
	private double lastEncoderReading;
	private double wheelSize = 4.0;// wheel size in inches
	private double calculatedEncoderCount = 0.0;
	private double lastCalculatedEncoderCount = 0.0;
	private double wheelToEncoderRatio = 3.2;
	private double moduleRotationRatio = 2.3;
	private int countsPerRev;
	private String name;
	private double time = 0;

	public SwerveModule(TalonSRX drive, Encoder driveEnc, PWMTalonSRX dir, AbsoluteEncoder dirEnc, int countsPerRev, double forwardAng, String name) {
		this.driveMotor = drive;
		this.driveEncoder = driveEnc;
		this.directionMotor = dir;
		this.directionEncoder = dirEnc;
		this.PIDController = new PositionWheel(this);
		this.forwardAngle = forwardAng;
		this.countsPerRev = countsPerRev;
		this.name = name;
		
		/* The following java example limits the current to 10 amps whenever the current has exceeded 15 amps for 100 Ms */
		this.driveMotor.configContinuousCurrentLimit(50, 0);
		this.driveMotor.configPeakCurrentLimit(75, 0);
		this.driveMotor.configPeakCurrentDuration(100, 0);
		this.driveMotor.enableCurrentLimit(true);
		
		this.driveMotor.configOpenloopRamp(1, 0);

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
			
			double fieldAngle = Gyro.normalizeAngle(currentAngle + Robot.gyro.getGyroAngle());
			
//			SmartDashboard.putNumber("Field Angle " + this.name, fieldAngle);
			
			fieldX += Math.cos(Math.toRadians(Gyro.toUnitCircleAngle(fieldAngle))) * ((((calculatedEncoderCount - lastCalculatedEncoderCount) / countsPerRev) / wheelToEncoderRatio) * (Math.PI * wheelSize));// (calculatedEncoderCount - lastCalculatedEncoderCount);
			fieldY += Math.sin(Math.toRadians(Gyro.toUnitCircleAngle(fieldAngle))) * ((((calculatedEncoderCount - lastCalculatedEncoderCount) / countsPerRev) / wheelToEncoderRatio) * (Math.PI * wheelSize));
//			SmartDashboard.putNumber("Module Field Angle: " + this.name, Math.toRadians(Robot.gyro.toUnitCircleAngle(fieldAngle))/Math.PI);

			lastAngle = currentAngle;
			lastEncoderReading = driveEncoder.get();
			lastCalculatedEncoderCount = calculatedEncoderCount;
			SmartDashboard.putNumber("Delta Time" + this.name, Timer.getFPGATimestamp() - time);
			time = Timer.getFPGATimestamp();

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
		
		targetSpeed *= 0.8;

		speed = targetSpeed;
		driveMotor.set(ControlMode.PercentOutput, targetSpeed);
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
