package org.usfirst.frc.team6560.robot;

import org.usfirst.frc.team6560.robot.RobotMap.Logitech;
import org.usfirst.frc.team6560.robot.RobotMap.Xbox;
import org.usfirst.frc.team6560.robot.RobotMap.XboxDrive;
import org.usfirst.frc.team6560.robot.commands.AutoTimedMotor;
import org.usfirst.frc.team6560.robot.commands.AutoVisionAlign;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public final Joystick logitech;
	public final Joystick xbox;
	public final Joystick xboxDrive;
	
	private final Button autoVisionAlignButton;
	private final Button spinButton;
	public OI() {
		logitech = new Joystick(Logitech.ID);
		xbox = new Joystick(Xbox.ID);
		xboxDrive = new Joystick(XboxDrive.ID);

		autoVisionAlignButton = new JoystickButton(xboxDrive, RobotMap.XboxDrive.BUTTON_B);
		autoVisionAlignButton.whenPressed(new AutoVisionAlign());

		spinButton = new JoystickButton(xboxDrive, RobotMap.XboxDrive.BUTTON_Y);
		spinButton.whenActive(
			new AutoTimedMotor(
				() -> Robot.hatchThing.setMotor(0.5),
				() -> Robot.hatchThing.setMotor(0),
				600
			)
		);
	}

	public boolean getTrigger(Joystick x, int triggerAxis) {
		return Math.abs(x.getRawAxis(triggerAxis)) > 0.5;
	}
}
