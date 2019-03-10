package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualHatchThing extends Command {
    public ManualHatchThing() {
    	requires(Robot.hatchThing);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.hatchThing.setShoot(false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.oi.xbox.getRawButton(RobotMap.Xbox.BUTTON_A)||Robot.oi.logitech.getRawButton(RobotMap.Logitech.BUTTON_3) || Robot.oi.xboxDrive.getRawButton(RobotMap.XboxDrive.BUTTON_A)) {
    		Robot.hatchThing.setShoot(true);
    	} else {
    		Robot.hatchThing.setShoot(false);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }
    

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
