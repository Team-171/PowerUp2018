/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team171.robot;

import org.usfirst.frc.team171.RobotParts.AbsoluteEncoder;
import org.usfirst.frc.team171.RobotParts.SwerveModule;
import org.usfirst.frc.team171.robot.subsystems.Elevator;
import org.usfirst.frc.team171.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	public static TalonSRX driveLeftFrontMotor;
	public static TalonSRX driveLeftBackMotor;
	public static TalonSRX driveRightFrontMotor;
	public static TalonSRX driveRightBackMotor;
	
	public static PWMTalonSRX driveLeftFrontDirMotor;
	public static PWMTalonSRX driveLeftBackDirMotor;
	public static PWMTalonSRX driveRightFrontDirMotor;
	public static PWMTalonSRX driveRightBackDirMotor;
	
	public static PWMTalonSRX liftMotorRight;
	public static PWMTalonSRX liftMotorLeft;
	
	public static PWMTalonSRX leftArmMotor;
	public static PWMTalonSRX rightArmMotor;

	public static AbsoluteEncoder leftFrontDirEncoder;
	public static AbsoluteEncoder leftBackDirEncoder;
	public static AbsoluteEncoder rightFrontDirEncoder;
	public static AbsoluteEncoder rightBackDirEncoder;
	
	public static Encoder leftFrontEncoder;
	public static Encoder leftBackEncoder;
	public static Encoder rightFrontEncoder;
	public static Encoder rightBackEncoder;
	public static AnalogInput elevatorPot;
	
	public static SwerveModule leftFrontSwerve;
	public static SwerveModule leftBackSwerve;
	public static SwerveModule rightFrontSwerve;
	public static SwerveModule rightBackSwerve;
	
	public static Solenoid shifter;
	
	public static DigitalOutput mainLEDControl;
	public static DigitalOutput heightControl;
	
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	
	public static void init(){		
		driveLeftFrontMotor = new TalonSRX(1);
		driveLeftBackMotor = new TalonSRX(2);
		driveRightFrontMotor = new TalonSRX(3);
		driveRightBackMotor = new TalonSRX(4);
				
		driveLeftFrontDirMotor = new PWMTalonSRX(0);
		driveLeftBackDirMotor = new PWMTalonSRX(1);
		driveRightFrontDirMotor = new PWMTalonSRX(2);
		driveRightBackDirMotor = new PWMTalonSRX(3);
		
		leftArmMotor = new PWMTalonSRX(15);
		rightArmMotor = new PWMTalonSRX(16);
		
		liftMotorRight = new PWMTalonSRX(13);
		liftMotorRight.setInverted(true);
		liftMotorLeft = new PWMTalonSRX(14);
		liftMotorLeft.setInverted(false);
		
		leftFrontDirEncoder = new AbsoluteEncoder(0);
		leftBackDirEncoder = new AbsoluteEncoder(3);
		rightFrontDirEncoder = new AbsoluteEncoder(6);
		rightBackDirEncoder = new AbsoluteEncoder(10);
		
		leftFrontEncoder = new Encoder(1, 2);
		leftBackEncoder = new Encoder(4, 5);
		rightFrontEncoder = new Encoder(7, 8);
		rightBackEncoder = new Encoder(11, 12);
		elevatorPot = new AnalogInput(0);
		
//		shifter = new Solenoid(4);
		mainLEDControl = new DigitalOutput(21);
		mainLEDControl.setPWMRate(2000);
		
		heightControl = new DigitalOutput(22);
		heightControl.setPWMRate(2000);
		
		leftFrontSwerve = new SwerveModule(driveLeftFrontMotor, leftFrontEncoder, driveLeftFrontDirMotor, leftFrontDirEncoder, 256, 79.7, "Front Left");
		
		leftBackSwerve = new SwerveModule(driveLeftBackMotor, leftBackEncoder, driveLeftBackDirMotor, leftBackDirEncoder, 128, 286.1, "Back Left");
		
		rightFrontSwerve = new SwerveModule(driveRightFrontMotor, rightFrontEncoder, driveRightFrontDirMotor, rightFrontDirEncoder, 128, 349.6, "Front Right");

		rightBackSwerve = new SwerveModule(driveRightBackMotor, rightBackEncoder, driveRightBackDirMotor, rightBackDirEncoder, 256, 70.4, "Back Right");
		
		
	}
}
