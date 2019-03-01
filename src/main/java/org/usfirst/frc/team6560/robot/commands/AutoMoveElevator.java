package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.util.ElevatorLevel;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoMoveElevator extends Command {
    public static final double ELEVATOR_MAX_OUTPUT = 0.4;
    
    public static final int[] STEP_POSITIONS = {4096 * 20, 4096 * 30, 4096 * 40, 4096 * 50};

    public enum Direction {
        UP,
        DOWN
    }

    private static int curTargetStep = -1;

    private final Direction dir;

    public AutoMoveElevator(Direction dir) {
        requires(Robot.elevator);
        setInterruptible(true);
        this.dir = dir;
    }

    public static void clearTargetStep() {
        curTargetStep = -1;
    }

    private int getNewTargetStep() {
        int curPos = Robot.elevator.getLevel1().getPosition() + Robot.elevator.getLevel2().getPosition();

        for (int i = 0; i < STEP_POSITIONS.length; ++i) {
            int stepPos = STEP_POSITIONS[i];
            if (curPos < stepPos) {
                if (dir == Direction.DOWN) {
                    return Math.max(0, i - 1);
                } else {
                    return i;
                }
            }
        }

        return STEP_POSITIONS.length - 1;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        if (curTargetStep == -1) {
            curTargetStep = getNewTargetStep();
        } else {
            if (dir == Direction.DOWN) {
                curTargetStep = Math.max(0, curTargetStep - 1);
            } else {
                curTargetStep = Math.min(STEP_POSITIONS.length - 1, curTargetStep + 1);
            }
        }

    	Robot.elevator.getLevel1().setOutput(0);
    	Robot.elevator.getLevel2().setOutput(0);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		double lv1 = Robot.oi.xbox.getRawAxis(RobotMap.Xbox.RIGHT_JOY_Y);
		double lv2 = Robot.oi.xbox.getRawAxis(RobotMap.Xbox.LEFT_JOY_Y);

		if(Math.abs(lv1) > 0.1){
			Robot.elevator.getLevel1().setOutput(lv1);
		}else{
			Robot.elevator.getLevel1().setOutput(0);
		}

		if(Math.abs(lv2) > 0.1){
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
