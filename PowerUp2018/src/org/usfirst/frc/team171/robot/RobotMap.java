/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team171.robot;

import org.usfirst.frc.team171.robot.subsystems.AbsoluteEncoder;

import edu.wpi.first.wpilibj.PWMTalonSRX;

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
	public static PWMTalonSRX driveLeftFrontMotor;
	public static PWMTalonSRX driveLeftBackMotor;
	public static PWMTalonSRX driveRightFrontMotor;
	public static PWMTalonSRX driveRightBackMotor;
	
	public static PWMTalonSRX driveLeftFrontDirMotor;
	public static PWMTalonSRX driveLeftBackDirMotor;
	public static PWMTalonSRX driveRightFrontDirMotor;
	public static PWMTalonSRX driveRightBackDirMotor;
	
	public static AbsoluteEncoder leftFrontEncoder;
	public static AbsoluteEncoder leftBackEncoder;
	public static AbsoluteEncoder rightFrontEncoder;
	public static AbsoluteEncoder rightBackEncoder;
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	
	public static void init(){
		driveLeftFrontMotor = new PWMTalonSRX(0);
		driveLeftBackMotor = new PWMTalonSRX(1);
		driveRightFrontMotor = new PWMTalonSRX(2);
		driveRightBackMotor = new PWMTalonSRX(3);
		
		driveLeftFrontDirMotor = new PWMTalonSRX(4);
		driveLeftBackDirMotor = new PWMTalonSRX(5);
		driveRightFrontDirMotor = new PWMTalonSRX(6);
		driveRightBackDirMotor = new PWMTalonSRX(7);
		
		leftFrontEncoder = new AbsoluteEncoder(0);
		leftBackEncoder = new AbsoluteEncoder(1);
		rightFrontEncoder = new AbsoluteEncoder(2);
		rightBackEncoder = new AbsoluteEncoder(3);
	}
}
