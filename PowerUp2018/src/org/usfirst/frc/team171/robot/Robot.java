/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team171.robot;

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

import org.usfirst.frc.team171.Autonomous.DoNothing;
import org.usfirst.frc.team171.Autonomous.DriveStraightLeft;
import org.usfirst.frc.team171.Autonomous.DriveStraightRight;
import org.usfirst.frc.team171.Autonomous.StartFromLeft;
import org.usfirst.frc.team171.Autonomous.StartFromMiddle;
import org.usfirst.frc.team171.Autonomous.StartFromRight;
import org.usfirst.frc.team171.robot.commands.SetRobotPosition;
import org.usfirst.frc.team171.robot.subsystems.DriveTrain;
import org.usfirst.frc.team171.robot.subsystems.Gyro;
import org.usfirst.frc.team171.robot.subsystems.JeVois;

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
	public static Preferences prefs;
	public static boolean joystickRunning = true;
	
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		try {
			imu = new AHRS(SPI.Port.kMXP);
//			imu = new AHRS(SerialPort.Port.kUSB1);
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
		oi = new OI();
		driveTrain = new DriveTrain();
		m_chooser.addDefault("Start From Left", new StartFromLeft());
		m_chooser.addObject("Start from middle", new StartFromMiddle());
		m_chooser.addObject("Start from right", new StartFromRight());
		m_chooser.addObject("Do nothing", new DoNothing());
		m_chooser.addObject("Drive straight left", new DriveStraightLeft());
		m_chooser.addObject("Drive straight right", new DriveStraightRight());

		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
		prefs = Preferences.getInstance();
				
        RobotMap.leftFrontSwerve.PIDController.setPIDF(0.015, 0.001, 0, 0);
		RobotMap.leftBackSwerve.PIDController.setPIDF(0.015, 0.001, 0, 0);
		RobotMap.rightFrontSwerve.PIDController.setPIDF(0.015, 0.001, 0, 0);
		RobotMap.rightBackSwerve.PIDController.setPIDF(0.015, 0.001, 0, 0);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		prefs.putDouble("P", RobotMap.leftFrontSwerve.PIDController.getPIDController().getP());
		prefs.putDouble("I", RobotMap.leftFrontSwerve.PIDController.getPIDController().getI());
		prefs.putDouble("D", RobotMap.leftFrontSwerve.PIDController.getPIDController().getD());
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
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateStatus();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		
		joystickRunning = true;
		
//		RobotMap.leftFrontSwerve.PIDController.setPIDF(prefs.getDouble("P", 0), prefs.getDouble("I", 0), prefs.getDouble("D", 0), 0);
//		RobotMap.leftBackSwerve.PIDController.setPIDF(prefs.getDouble("P", 0), prefs.getDouble("I", 0), prefs.getDouble("D", 0), 0);
//		RobotMap.rightFrontSwerve.PIDController.setPIDF(prefs.getDouble("P", 0), prefs.getDouble("I", 0), prefs.getDouble("D", 0), 0);
//		RobotMap.rightBackSwerve.PIDController.setPIDF(prefs.getDouble("P", 0), prefs.getDouble("I", 0), prefs.getDouble("D", 0), 0);
//		RobotMap.driveLeftFrontDirMotor.set(-1);
		
//		RobotMap.leftFrontSwerve.PIDController.disable();
//		RobotMap.leftBackSwerve.PIDController.disable();
//		RobotMap.rightFrontSwerve.PIDController.disable();
//		RobotMap.rightBackSwerve.PIDController.disable();
		gyro.resetGyro();
		gyro.setTargetAngle(gyro.getGyroAngle());
		new SetRobotPosition(0, 0).start();
		
//		new SetRobotPosition(0, 20).start();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateStatus();
		
//		RobotMap.driveLeftFrontDirMotor.set(oi.gamepad.getY());
//		RobotMap.driveLeftBackDirMotor.set(oi.gamepad.getY());
//		RobotMap.driveRightFrontDirMotor.set(oi.gamepad.getY());
//		RobotMap.driveRightBackDirMotor.set(oi.gamepad.getY());
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void updateStatus(){
//		SmartDashboard.putNumber("encoder", RobotMap.leftFrontDirEncoder.getAngle());
		gyro.updateStatus();
		driveTrain.updateStatus();

		SmartDashboard.putNumber("Front Left Angle", RobotMap.leftFrontSwerve.directionEncoder.getAngle());
		SmartDashboard.putNumber("Back Left Angle", RobotMap.leftBackSwerve.directionEncoder.getAngle());
		SmartDashboard.putNumber("Front Right Angle", RobotMap.rightFrontSwerve.directionEncoder.getAngle());
		SmartDashboard.putNumber("Back Right Angle", RobotMap.rightBackSwerve.directionEncoder.getAngle());
		SmartDashboard.putNumber("Encoder Front", RobotMap.rightFrontEncoder.get());
		SmartDashboard.putNumber("Encoder Back", RobotMap.rightBackEncoder.get());
		SmartDashboard.putNumber("Displacement", imu.getVelocityX());
//		SmartDashboard.putBoolean("Rotating", driveTrain.rotating);
//		
//		SmartDashboard.putNumber("Left Front Encoder", RobotMap.leftFrontSwerve.driveEncoder.get());
//		
//		SmartDashboard.putNumber("Angle Number", gyro.getAngleError(270));
	}
}
