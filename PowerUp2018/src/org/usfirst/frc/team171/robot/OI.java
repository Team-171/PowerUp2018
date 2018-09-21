/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team171.robot;

import org.usfirst.frc.team171.Autonomous.StartFromLeft;
import org.usfirst.frc.team171.Autonomous.Actions.WayPoint;
import org.usfirst.frc.team171.RobotMotion.SetElevatorPosition;
import org.usfirst.frc.team171.RobotMotion.SetIntakeSpeed;
import org.usfirst.frc.team171.robot.commands.NoDriveMode;
import org.usfirst.frc.team171.robot.commands.ResetGyro;
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
	public Joystick operator_gamepad;
	public static JoystickButton noDriveMode;
	public static JoystickButton swerveCalibrationMode;
	public static JoystickButton fieldOriented;
	public static JoystickButton testPosition;
	public static JoystickButton elevatorDown;
	public static JoystickButton elevatorUp;
	public static JoystickButton elevatorMiddle;
	public static JoystickButton resetGyro;
	
	public OI(){
		gamepad = new Joystick(0);
		operator_gamepad = new Joystick(1);

		fieldOriented = new JoystickButton(gamepad, 5);
		fieldOriented.whenPressed(new SetFieldOriented(false));
		fieldOriented.whenReleased(new SetFieldOriented(true));
		
		resetGyro = new JoystickButton(gamepad, 2);
	    resetGyro.whenPressed(new ResetGyro());
		
		if(Robot.oneController)
		{
			elevatorUp = new JoystickButton(gamepad, 4);
			elevatorUp.whenPressed(new SetElevatorPosition(30));
			
			elevatorMiddle = new JoystickButton(gamepad, 3);
			elevatorMiddle.whenPressed(new SetElevatorPosition(15));
			
			elevatorDown = new JoystickButton(gamepad, 1);
			elevatorDown.whenPressed(new SetElevatorPosition(3));
		}
		else
		{
			elevatorUp = new JoystickButton(operator_gamepad, 4);
			elevatorUp.whenPressed(new SetElevatorPosition(30));

			elevatorMiddle = new JoystickButton(operator_gamepad, 3);
			elevatorMiddle.whenPressed(new SetElevatorPosition(15));
			
			elevatorDown = new JoystickButton(operator_gamepad, 1);
			elevatorDown.whenPressed(new SetElevatorPosition(3));
		}
	    
//		testPosition = new JoystickButton(gamepad, 6);
//		testPosition.whenPressed(new SetRobotPosition(0, 0));
//		testPosition.whenPressed(new StartFromLeft());
//		testPosition.whenReleased(new WayPoint(60, 100, 0, .4));
		
//		noDriveMode = new JoystickButton(gamepad, 5);
//		noDriveMode.whenPressed(new NoDriveMode());

//		swerveCalibrationMode = new JoystickButton(gamepad, 6);
//		noDriveMode.whenPressed(new SwerveCalibrationMode());
	}
}
