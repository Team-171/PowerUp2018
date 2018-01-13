package org.usfirst.frc.team171.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;

public class AbsoluteEncoder {
	double port;
	Counter input;
	
	public AbsoluteEncoder(double port){
		this.port = port;
		input = new Counter((int) port);
		input.setSemiPeriodMode(true);
	}
	
	public double getAngle(){
		return (input.getPeriod() * 250) * 360;
//		return input.pidGet();
	}
}
