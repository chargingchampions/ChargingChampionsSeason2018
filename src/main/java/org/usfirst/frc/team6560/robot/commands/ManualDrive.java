package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualDrive extends Command {
	public static final double TURN_SPEED = 0.85;
    public static final double MAX_SPEED = 15;
    
    private static short driveMode = 1;

    public ManualDrive() {
        requires(Robot.driveTrain);
        setInterruptible(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Running JoystickDrive command...");
    	Robot.driveTrain.setManual();
    	Robot.driveTrain.stop();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	

        if(Robot.oi.logitech.getRawButton(RobotMap.Logitech.BUTTON_7)){
            driveMode = 1;
        }

        if(Robot.oi.logitech.getRawButton(RobotMap.Logitech.BUTTON_8)){
            driveMode = 2;
        }

    	double x = -Robot.oi.logitech.getX();
        double y = -Robot.oi.logitech.getY();
    	
    	double multiplier = -(Robot.oi.logitech.getThrottle() - 1.0) / 2 * MAX_SPEED;
        double radius = Math.sqrt(x*x + y*y);
        double t = Math.atan2(y, x);

        if (radius < 0.1) {
            Robot.driveTrain.stop();
            return;
        }

        double s = (TURN_SPEED / 2) - (TURN_SPEED / 2) * (multiplier / 20);
        
        if (s > 0.5 || s < 0.0) throw new RuntimeException("Uh");

        double cosSign = Math.copySign(1.0, Math.cos(t));
        double sinSign = Math.copySign(1.0, Math.sin(t));
        double tanSign = Math.copySign(1.0, Math.tan(t));

        double funcVal = Math.cos(2*t);

        double lFactor = -cosSign * (s + tanSign * 0.5) * funcVal - cosSign * s + sinSign * 0.5;
        double rFactor = cosSign * (s - tanSign * 0.5) * funcVal + cosSign * s + sinSign * 0.5;
        
        if(driveMode == 1){
            Robot.driveTrain.setVelL(lFactor * radius * multiplier);
            Robot.driveTrain.setVelR(rFactor * radius * multiplier);
        }
        if(driveMode == 2){
            Robot.driveTrain.motorDrive.arcadeDrive(-y * multiplier, -x * TURN_SPEED);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
