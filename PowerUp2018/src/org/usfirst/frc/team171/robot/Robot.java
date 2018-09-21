/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team171.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.text.StyleContext.SmallAttributeSet;

import org.usfirst.frc.team171.Autonomous.DoNothing;
import org.usfirst.frc.team171.Autonomous.DriveStraightLeft;
import org.usfirst.frc.team171.Autonomous.DriveStraightLeftt;
import org.usfirst.frc.team171.Autonomous.DriveStraightRight;
import org.usfirst.frc.team171.Autonomous.DriveStraightttttt;
import org.usfirst.frc.team171.Autonomous.StartFromLeft;
import org.usfirst.frc.team171.Autonomous.StartFromMiddle;
import org.usfirst.frc.team171.Autonomous.StartFromRight;
import org.usfirst.frc.team171.robot.commands.SetRobotPosition;
import org.usfirst.frc.team171.robot.subsystems.DriveTrain;
import org.usfirst.frc.team171.robot.subsystems.Elevator;
import org.usfirst.frc.team171.robot.subsystems.Gyro;
import org.usfirst.frc.team171.robot.subsystems.Intake;
import org.usfirst.frc.team171.robot.subsystems.JeVois;
import org.usfirst.frc.team171.robot.subsystems.LEDs;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI oi;
	public static DriveTrain driveTrain;
	public static AHRS imu;
	public static Gyro gyro;
	public static Elevator elevator;
	public static LEDs leds;
	public static Intake intake;
	public static Preferences prefs;
	public static boolean joystickRunning = true;
	public static edu.wpi.first.wpilibj.networktables.NetworkTable table;
	private SendableChooser<Integer> autoSelection;
	public Command autoCommand;
	public UsbCamera visionCamera;
	public static boolean oneController = false;

	
	Command m_autonomousCommand;
//	SendableChooser<int> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try {
			imu = new AHRS(SPI.Port.kMXP);
			// imu = new AHRS(SerialPort.Port.kUSB1);
			imu.setPIDSourceType(PIDSourceType.kDisplacement);

		} catch (Exception ex) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
		if (imu != null) {
			LiveWindow.addSensor("IMU", "Gyro", imu);
		}
		Timer.delay(3);

		imu.reset();

		gyro = new Gyro();
		RobotMap.init();
		leds = new LEDs(RobotMap.mainLEDControl, RobotMap.heightControl);
		oi = new OI();
		driveTrain = new DriveTrain(RobotMap.leftFrontSwerve, RobotMap.leftBackSwerve, RobotMap.rightFrontSwerve,
				RobotMap.rightBackSwerve);
		elevator = new Elevator(RobotMap.liftMotorLeft, RobotMap.liftMotorRight, RobotMap.elevatorPot);
		intake = new Intake(RobotMap.leftArmMotor, RobotMap.rightArmMotor);

//		visionCamera = CameraServer.getInstance().startAutomaticCapture(0);
//		visionCamera.setResolution(320, 240);
//		visionCamera.setBrightness(8);
//		visionCamera.setExposureManual(1);
//		visionCamera.setWhiteBalanceManual(1);
		
		autoSelection = new SendableChooser<Integer>();
		
		autoSelection.addDefault("Start From Left", 0);
		autoSelection.addObject("Start from middle", 1);
		autoSelection.addObject("Start from right", 2);
		autoSelection.addObject("Do nothing", 3);
		autoSelection.addObject("Drive straight left", 4);
		autoSelection.addObject("Drive straight right", 5);
		autoSelection.addObject("Drive straight Closed", 6);
		
//		m_chooser.addDefault("Start From Left", new StartFromLeft());
//		m_chooser.addObject("Start from middle", new StartFromMiddle());
//		m_chooser.addObject("Start from right", new StartFromRight());
//		m_chooser.addObject("Do nothing", new DoNothing());
//		m_chooser.addObject("Drive straight left", new DriveStraightLeft());
//		m_chooser.addObject("Drive straight right", new DriveStraightRight());

		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", autoSelection);
		prefs = Preferences.getInstance();

		RobotMap.leftFrontSwerve.getPID().setPIDF(0.015, 0.003, 0.002, 0.00);
		RobotMap.leftBackSwerve.getPID().setPIDF(0.015, 0.003, 0.002, 0.00);
		RobotMap.rightFrontSwerve.getPID().setPIDF(0.009, 0.001, 0.002, 0.00);
		RobotMap.rightBackSwerve.getPID().setPIDF(0.015, 0.003, 0.002, 0.00);

		table = edu.wpi.first.wpilibj.networktables.NetworkTable.getTable("subWaypoint");
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		prefs.putDouble("P", RobotMap.leftFrontSwerve.getPID().getPIDController().getP());
		prefs.putDouble("I", RobotMap.leftFrontSwerve.getPID().getPIDController().getI());
		prefs.putDouble("D", RobotMap.leftFrontSwerve.getPID().getPIDController().getD());
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateStatus();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
//		m_autonomousCommand = m_chooser.getSelected();
		
		switch(autoSelection.getSelected())
		{
		case 0:
			autoCommand = new StartFromLeft();
			break;
			
		case 1:
			autoCommand = new StartFromMiddle();
			break;
			
		case 2:
			autoCommand = new StartFromRight();
			break;
			
		case 3:
			autoCommand = new DoNothing();
			break;
			
		case 4:
			autoCommand = new DriveStraightLeft();
			break;
			
		case 5:
			autoCommand = new DriveStraightLeft();
			break;
			
		case 6:
			autoCommand = new DriveStraightttttt();
			break;
		}
		
//		RobotMap.shifter.set(true);
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// new DriveStraightLeft().start();
		// schedule the autonomous command (example)
		if (autoCommand != null) {
			autoCommand.start();
		}
		
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putBoolean("Auto Running", autoCommand.isRunning());
		SmartDashboard.putString("Auto Name", autoCommand.getName());
		updateStatus();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autoCommand != null) {
			autoCommand.cancel();
		}

		joystickRunning = true;

		// RobotMap.leftFrontSwerve.PIDController.setPIDF(prefs.getDouble("P",
		// 0), prefs.getDouble("I", 0), prefs.getDouble("D", 0), 0);
		// RobotMap.leftBackSwerve.PIDController.setPIDF(prefs.getDouble("P",
		// 0), prefs.getDouble("I", 0), prefs.getDouble("D", 0), 0);
		// RobotMap.rightFrontSwerve.PIDController.setPIDF(prefs.getDouble("P",
		// 0), prefs.getDouble("I", 0), prefs.getDouble("D", 0), 0);
		// RobotMap.rightBackSwerve.PIDController.setPIDF(prefs.getDouble("P",
		// 0), prefs.getDouble("I", 0), prefs.getDouble("D", 0), 0);
		// RobotMap.driveLeftFrontDirMotor.set(-1);

		// RobotMap.leftFrontSwerve.PIDController.disable();
		// RobotMap.leftBackSwerve.PIDController.disable();
		// RobotMap.rightFrontSwerve.PIDController.disable();
		// RobotMap.rightBackSwerve.PIDController.disable();
		gyro.resetGyro();
		gyro.setTargetAngle(gyro.getGyroAngle());
		// new SetRobotPosition(0, 0).start();
//		RobotMap.shifter.set(true);

		// new SetRobotPosition(0, 20).start();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateStatus();
		joystickRunning = true;
		// RobotMap.driveLeftFrontDirMotor.set(oi.gamepad.getY());
		// RobotMap.driveLeftBackDirMotor.set(oi.gamepad.getY());
		// RobotMap.driveRightFrontDirMotor.set(oi.gamepad.getY());
		// RobotMap.driveRightBackDirMotor.set(oi.gamepad.getY());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	public static String getGSM() {
		String message = "";
		int counter = 0, maxCounts = 10;

		do {
			message = DriverStation.getInstance().getGameSpecificMessage();

			if (message == null) {
				message = "";
			}

			try {
				Thread.sleep(500);
			} catch (InterruptedException ex) {
			}

			counter++;
		} while (message.isEmpty() && counter < maxCounts);

		return message;
	}

	public static double round(double valueToRound, int numberOfDecimalPlaces) {
		double multipicationFactor = Math.pow(10, numberOfDecimalPlaces);
		double interestedInZeroDPs = valueToRound * multipicationFactor;
		return Math.round(interestedInZeroDPs) / multipicationFactor;
	}

	public void updateStatus() {
		// SmartDashboard.putNumber("encoder",
		// RobotMap.leftFrontDirEncoder.getAngle());
		gyro.updateStatus();
		driveTrain.updateStatus();
		elevator.updateStatus();
		leds.updateStatus();

		SmartDashboard.putNumberArray("Swerve Angles",
				new double[] { round(RobotMap.leftFrontSwerve.getAbsEnc().getAngle(), 2),
						round(RobotMap.leftBackSwerve.getAbsEnc().getAngle(), 2),
						round(RobotMap.rightFrontSwerve.getAbsEnc().getAngle(), 2),
						round(RobotMap.rightBackSwerve.getAbsEnc().getAngle(), 2) });

		// SmartDashboard.putNumberArray("Swerve Angles: Left",
		// new double[] {
		// round(RobotMap.leftFrontSwerve.directionEncoder.getAngle(), 2),
		// round(RobotMap.leftBackSwerve.directionEncoder.getAngle(), 2) });
		//
		// SmartDashboard.putNumberArray("Swerve Angles: Right",
		// new double[] {
		// round(RobotMap.rightFrontSwerve.directionEncoder.getAngle(), 2),
		// round(RobotMap.rightBackSwerve.directionEncoder.getAngle(), 2) });

		SmartDashboard.putNumberArray("Encoders", new double[] { RobotMap.leftFrontEncoder.get(),
				RobotMap.leftBackEncoder.get(), RobotMap.rightFrontEncoder.get(), RobotMap.rightBackEncoder.get() });

		SmartDashboard.putNumber("Back Left Drive", RobotMap.driveLeftBackMotor.getMotorOutputPercent());
		// SmartDashboard.putNumber("Front Left Angle",
		// RobotMap.leftFrontSwerve.directionEncoder.getAngle());
		// SmartDashboard.putNumber("Back Left Angle",
		// RobotMap.leftBackSwerve.directionEncoder.getAngle());
		// SmartDashboard.putNumber("Front Right Angle",
		// RobotMap.rightFrontSwerve.directionEncoder.getAngle());
		// SmartDashboard.putNumber("Back Right Angle",
		// RobotMap.rightBackSwerve.directionEncoder.getAngle());
		//
		// SmartDashboard.putNumber("Encoder Front Left",
		// RobotMap.leftFrontEncoder.get());
		// SmartDashboard.putNumber("Encoder Back Left",
		// RobotMap.leftBackEncoder.get());
		//
		// SmartDashboard.putNumber("Encoder Front Right",
		// RobotMap.rightFrontEncoder.get());
		// SmartDashboard.putNumber("Encoder Back Right",
		// RobotMap.rightBackEncoder.get());

		// SmartDashboard.putNumber("Displacement", imu.getVelocityX());

		// SmartDashboard.putBoolean("Rotating", driveTrain.rotating);
		//
		// SmartDashboard.putNumber("Left Front Encoder",
		// RobotMap.leftFrontSwerve.driveEncoder.get());
		//
		// SmartDashboard.putNumber("Angle Number", gyro.getAngleError(270));
	}
}
