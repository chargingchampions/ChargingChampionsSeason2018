package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.util.ElevatorLevel;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualElevatorLimitSwitch extends Command {
	public static final double ELEVATOR_MAX_OUTPUT = 0.4;

    public ManualElevatorLimitSwitch() {
    	requires(Robot.elevator);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.elevator.getLevel1().setOutput(0);
    	Robot.elevator.getLevel2().setOutput(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		double lv1 = -Robot.oi.xbox.getRawAxis(RobotMap.Xbox.RIGHT_JOY_Y);
		double lv2 = Robot.oi.xbox.getRawAxis(RobotMap.Xbox.LEFT_JOY_Y);

		if(Math.abs(lv1) > 0.1){
			setElevator(lv1,Robot.elevator.getLevel1());
		}else{
			setElevator(0,Robot.elevator.getLevel1());
		}

		if(Math.abs(lv2) > 0.1){
			setElevator(lv2,Robot.elevator.getLevel2());
		}else{
			setElevator(0,Robot.elevator.getLevel2());
		}

		System.out.println(Robot.elevator.getLevel2().getLimTop());
		System.out.println(Robot.elevator.getLevel2().getLimBottom());

    	// double y = Robot.oi.xbox.getRawAxis(RobotMap.Xbox.RIGHT_JOY_Y);
    	
    	// if (Math.abs(y) > 0.1) {
    	// 	setElevator(y);
    	// } else {
    	// 	setElevator(0);
    	// }
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
    
    private void setElevator(double output, ElevatorLevel elevator) {    	
    	if (output >= 0) {
        	if (elevator.getLimTop()) {
				elevator.setOutput(output);
			}else{
				elevator.setOutput(0);
			}
    	} else {
        	if (elevator.getLimBottom()) {
        		elevator.setOutput(output);
			}else{
        		elevator.setOutput(0);
			}
    	}
    }
}
