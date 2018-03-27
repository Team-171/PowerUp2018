package org.usfirst.frc.team171.RobotParts;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.PWMTalonSRX;

public class SeatMotor {

	private PWMTalonSRX m_controller;
	private int counts;
	private Counter counter;
	private int rate = 200;// Hz

	public SeatMotor(int controllerChannel, int encoderChannel) {
		this.m_controller = new PWMTalonSRX(controllerChannel);
		this.counter = new Counter(encoderChannel);
		
		Thread positionThread = new Thread(() -> trackPosition());
		positionThread.setDaemon(true);
		positionThread.setName("Custom Super Fast Loop");
		positionThread.start();
	}

	private void trackPosition() {
		int lastCount = 0;
		
		while (true) {
			if(this.m_controller.get()>0)
			{
				this.counts += this.counter.get() - lastCount;
			}
			
			if(this.m_controller.get()<0)
			{
				this.counts -= this.counter.get() - lastCount;
			}
			
			lastCount = this.counter.get();
			try {
				Thread.sleep(1000 / rate);
			} catch (InterruptedException ex) {
			}
		}
	}
	
	public void set(double speed){
		this.m_controller.set(speed);
	}
	
	public void reset(){
		this.counts = 0;
	}
	
	public double getAngle(){
		//TODO: Impliment actual calculation 
		return this.counts;
	}

}
