package org.usfirst.frc.team6560.robot.subsystems;

import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.commands.ManualElevatorPistons;
import org.usfirst.frc.team6560.robot.commands.ManualRearHatch;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ElevatorPistons extends Subsystem {
	private Solenoid extend;
	private Solenoid shoot;
	
	public ElevatorPistons() {
		extend = new Solenoid(RobotMap.CLIMB_EXTENSION_SOLENOID);
		
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        setDefaultCommand(new ManualElevatorPistons());
    }
    
    public void setExtend(boolean b) {
    	extend.set(b);
    }
    
}

