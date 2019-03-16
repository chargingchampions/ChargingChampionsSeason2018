package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.util.ElevatorLevel;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualElevatorLimitSwitch extends Command {

    public ManualElevatorLimitSwitch() {
    	requires(Robot.elevator);

	}
	
	public boolean isManuallyControlling() {
		double lv1 = Robot.oi.xbox.getRawAxis(RobotMap.Xbox.RIGHT_JOY_Y);
		double lv2 = Robot.oi.xbox.getRawAxis(RobotMap.Xbox.LEFT_JOY_Y);

		return Math.abs(lv1) > 0.2 || Math.abs(lv2) > 0.2;
	}

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getLevel1().setOutput(0);
    	Robot.elevator.getLevel2().setOutput(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		double lv1 = -Robot.oi.xbox.getRawAxis(RobotMap.Xbox.LEFT_JOY_Y);
		double lv2 = -Robot.oi.xbox.getRawAxis(RobotMap.Xbox.RIGHT_JOY_Y);

		if(Math.abs(lv1) > 0.25){
			Robot.elevator.getLevel1().setOutput(lv1);
		}else{
			Robot.elevator.getLevel1().setOutput(0);
		}

		if(Math.abs(lv2) > 0.25){
			Robot.elevator.getLevel2().setOutput(lv2);
		}else{
			Robot.elevator.getLevel2().setOutput(0);
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.elevator.getLevel1().setOutput(0);
    	Robot.elevator.getLevel2().setOutput(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
