package org.usfirst.frc.team6560.robot;

import org.usfirst.frc.team6560.robot.RobotMap.Logitech;
import org.usfirst.frc.team6560.robot.RobotMap.Xbox;
import org.usfirst.frc.team6560.robot.RobotMap.XboxDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public final Joystick logitech;
	public final Joystick xbox;
	public final Joystick xboxDrive;
	
	public OI() {
		logitech = new Joystick(Logitech.ID);
		xbox = new Joystick(Xbox.ID);
		xboxDrive = new Joystick(XboxDrive.ID);
	}

	public boolean getTrigger(Joystick x, int triggerAxis) {
		return Math.abs(x.getRawAxis(triggerAxis)) > 0.5;
	}
}
