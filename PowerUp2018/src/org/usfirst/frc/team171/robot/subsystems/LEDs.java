package org.usfirst.frc.team171.robot.subsystems;

import org.usfirst.frc.team171.robot.triggers.RunLEDs;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LEDs extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	private DigitalOutput m_mainLEDControl;
	private DigitalOutput m_heightControl;
	private final double minHeight = 2;
	private final double maxHeight = 30;

	public enum Mode {
		IDLE(10.0 / 100), HEIGHT_UP(20.0 / 100), HEIGHT_DOWN(30.0 / 100);

		private double m_dutyCycle;

		private Mode(double dutyCycle) {
			this.m_dutyCycle = dutyCycle;
		}

		public double dutyCycle() {
			return this.m_dutyCycle;
		}
	}

	public LEDs(DigitalOutput mainLEDControl, DigitalOutput heightControl) {
		this.m_mainLEDControl = mainLEDControl;
		this.m_heightControl = heightControl;

		m_mainLEDControl.enablePWM(0);
		m_heightControl.enablePWM(0);
	}
	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new RunLEDs());
	}

	public void setMode(LEDs.Mode mode) {
		switch (mode) {
		case IDLE:
			m_mainLEDControl.updateDutyCycle(Mode.IDLE.dutyCycle());
			break;

		case HEIGHT_UP:
			m_mainLEDControl.updateDutyCycle(Mode.HEIGHT_UP.dutyCycle());
			break;

		case HEIGHT_DOWN:
			m_mainLEDControl.updateDutyCycle(Mode.HEIGHT_DOWN.dutyCycle());
			break;
		}
	}

	public void setDutyCycle(double dutyCycle) {
		if (dutyCycle < 0) {
			dutyCycle = 0;
		}

		if (dutyCycle > 1) {
			dutyCycle = 1;
		}

		this.m_heightControl.updateDutyCycle(dutyCycle);
	}

	
	public void updateStatus(){
		SmartDashboard.putBoolean("LED Mode", m_mainLEDControl.get());
	}
	
}
