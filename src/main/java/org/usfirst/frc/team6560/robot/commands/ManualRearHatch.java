package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualRearHatch extends Command {
    public ManualRearHatch() {
    	requires(Robot.rearHatch);
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.rearHatch.setExtend(false);
    	Robot.rearHatch.setShoot(false);
        
        lastExtendButtonState2 = false;
    	lastExtendButtonState1 = false;
    	extendState = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.oi.xbox.getRawButton(RobotMap.Xbox.BUTTON_A)||Robot.oi.logitech.getRawButton(RobotMap.Logitech.BUTTON_3)) {
    		Robot.rearHatch.setShoot(true);
    	} else {
    		Robot.rearHatch.setShoot(false);
    	}
    	
    	boolean extendButtonState1 = Robot.oi.xbox.getRawButton(RobotMap.Xbox.BUTTON_X);
        boolean extendButtonState2 = Robot.oi.logitech.getRawButton(RobotMap.Logitech.BUTTON_4);
        
    	if ((extendButtonState1 && !lastExtendButtonState1)||(extendButtonState2 && !lastExtendButtonState2)) {
    		extendState = !extendState;
    		Robot.rearHatch.setExtend(extendState);
    	}
    	
        lastExtendButtonState1 = extendButtonState1;
        lastExtendButtonState2 = extendButtonState2;
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
    
    private boolean extendState;
    private boolean lastExtendButtonState1;
    private boolean lastExtendButtonState2;
}
