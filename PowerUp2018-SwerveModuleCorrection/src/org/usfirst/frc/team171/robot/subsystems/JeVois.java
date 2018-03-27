package org.usfirst.frc.team171.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class JeVois extends Subsystem {

    SerialPort JeVois;
    
    public JeVois () {
    	JeVois = new SerialPort(115200, SerialPort.Port.kUSB);
    }

    public void initDefaultCommand() {
    }
    
    public String read() {
    	return JeVois.readString();
    }
}

