/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team171.robot;

import org.usfirst.frc.team171.Autonomous.SubWaypoint;
import org.usfirst.frc.team171.robot.commands.NoDriveMode;
import org.usfirst.frc.team171.robot.commands.SetFieldOriented;
import org.usfirst.frc.team171.robot.commands.SetRobotPosition;
import org.usfirst.frc.team171.robot.commands.SwerveCalibrationMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public Joystick gamepad;
	public static JoystickButton noDriveMode;
	public static JoystickButton swerveCalibrationMode;
	public static JoystickButton fieldOriented;
	public static JoystickButton testPosition;
	
	public OI(){
		gamepad = new Joystick(0);

		fieldOriented = new JoystickButton(gamepad, 5);
		fieldOriented.whenPressed(new SetFieldOriented(false));
		fieldOriented.whenReleased(new SetFieldOriented(true));
		
		testPosition = new JoystickButton(gamepad, 6);
		testPosition.whenPressed(new SetRobotPosition(0, 0));
		testPosition.whenReleased(new SubWaypoint(60, 60, 90));
		
//		noDriveMode = new JoystickButton(gamepad, 5);
//		noDriveMode.whenPressed(new NoDriveMode());

//		swerveCalibrationMode = new JoystickButton(gamepad, 6);
//		noDriveMode.whenPressed(new SwerveCalibrationMode());
	}
}
