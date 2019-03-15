package org.usfirst.frc.team6560.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.commands.ManualHatchThing;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class HatchThing extends Subsystem {
	private Solenoid pusher;
    private Solenoid releaser;
    
    private TalonSRX motor;
	
	public HatchThing() {
		pusher = new Solenoid(RobotMap.HATCH_PUSHER_SOLENOID);
		releaser = new Solenoid(RobotMap.HATCH_RELEASER_SOLENOID);
		motor = new TalonSRX(uh);
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        setDefaultCommand(new ManualHatchThing());
    }
    
    public void setShoot(boolean b) {
        pusher.set(b);
        releaser.set(b);
    }
    
    public void setMotor(double output) {
        motor.set(ControlMode.PercentOutput, output);
    }
}

