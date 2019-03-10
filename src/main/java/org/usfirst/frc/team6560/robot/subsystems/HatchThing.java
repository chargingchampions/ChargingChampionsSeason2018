package org.usfirst.frc.team6560.robot.subsystems;

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
	
	public HatchThing() {
		pusher = new Solenoid(RobotMap.HATCH_PUSHER_SOLENOID);
		releaser = new Solenoid(RobotMap.HATCH_RELEASER_SOLENOID);
		
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
}

