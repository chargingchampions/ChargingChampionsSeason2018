package org.usfirst.frc.team6560.robot.util;

import org.usfirst.frc.team6560.robot.Robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorLevel {	
	ArrayList<WPI_TalonSRX> motors = new ArrayList<>();
	
	DigitalInput limTop;
	DigitalInput limBottom;

	public ElevatorLevel(int motorId, int limTopId, int limBottomId) {
		this(motorId, limTopId, limBottomId, false);
	}

	public ElevatorLevel(int motorId, int limTopId, int limBottomId, boolean inverted) {
		this(new int[]{motorId}, limTopId, limBottomId, inverted);
	}

	

	public ElevatorLevel(int[] motorIds, int limTopId, int limBottomId) {
		this(motorIds, limTopId, limBottomId, false);
	}
	
	public ElevatorLevel(int[] motorIds, int limTopId, int limBottomId, boolean inverted) {
		for (int id : motorIds) {
			motors.add(new WPI_TalonSRX(id));
		}

		for (WPI_TalonSRX motor : motors) {
			Robot.initializeMotorManual(motor, 0.2);
			motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
			motor.setInverted(inverted);
		}
		
		limTop = new DigitalInput(limTopId);
		limBottom = new DigitalInput(limBottomId);
	}
	
	public void update() {
		if (requested >= 0) {
        	if (getLimTop()) {
				for (WPI_TalonSRX motor : motors)
					motor.set(ControlMode.PercentOutput, requested);
			}else{
				for (WPI_TalonSRX motor : motors)
					motor.set(ControlMode.PercentOutput, 0);
			}
    	} else {
        	if (getLimBottom()) {
				for (WPI_TalonSRX motor : motors)
					motor.set(ControlMode.PercentOutput, requested);
			}else{
				for (WPI_TalonSRX motor : motors)
					motor.set(ControlMode.PercentOutput, 0);
			}
		}
		
		
	}
	
	public void setOutput(double output) {
		requested = output;
	}
	
	public boolean getLimTop() {
		return limTop.get();
	}
	
	public boolean getLimBottom() {
		return limBottom.get();
	}
	
	public int getPosition() {
		return motors.get(0).getSelectedSensorPosition(0);
	}
	
	public void setPosition(int pos) {
		for (WPI_TalonSRX motor : motors)
			motor.setSelectedSensorPosition(pos, 0, 30);
	}
	
	private double requested;
}
