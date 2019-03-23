package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.subsystems.DriveTrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ManualDrive extends Command {
    public static final int MAX_SPEED = 12;

    public static final int INITIAL_SPEED = 5;
    public static final double INITIAL_TURN_SPEED = 3.8;
    
    private int speed = INITIAL_SPEED;
    private double turn_speed = INITIAL_TURN_SPEED;
    private int lastPOV;

    public ManualDrive() {
        requires(Robot.driveTrain);
        setInterruptible(true);

        SmartDashboard.putNumber("Speed", speed);
        turn_speed = Robot.prefs.getDouble("turn_speed", INITIAL_TURN_SPEED);
        
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Running JoystickDrive command...");
        Robot.driveTrain.stop();

        speed = INITIAL_SPEED;
        SmartDashboard.putNumber("Speed", speed);
        turn_speed = Robot.prefs.getDouble("turn_speed", INITIAL_TURN_SPEED);

        lastPOV = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        int pov = Robot.oi.xboxDrive.getPOV();

        if (lastPOV == -1) {
            if (pov == 0) {
                speed += 1;
            } else if (pov == 180) {
                speed -= 1;
            }
        }

        lastPOV = pov;

        speed = Math.min(MAX_SPEED, Math.max(0, speed));
        SmartDashboard.putNumber("Speed", speed);

        double x = 0;
        double y = 0;

        x = -Robot.oi.xboxDrive.getRawAxis(RobotMap.XboxDrive.RIGHT_JOY_X);
        y = -Robot.oi.xboxDrive.getRawAxis(RobotMap.XboxDrive.LEFT_JOY_Y);
    	        
        double radius = Math.sqrt(x*x + y*y);
        double t = Math.atan2(y, x);

        double s = Math.min(turn_speed / (2.0*speed), 0.5);

        if (s < 0.1) {
            s = 0;
        }
        
        double cosSign = Math.copySign(1.0, Math.cos(t));
        double sinSign = Math.copySign(1.0, Math.sin(t));
        double tanSign = Math.copySign(1.0, Math.tan(t));

        double funcVal = Math.cos(2*t);

        double lFactor = -cosSign * (s + tanSign * 0.5) * funcVal - cosSign * s + sinSign * 0.5;
        double rFactor = cosSign * (s - tanSign * 0.5) * funcVal + cosSign * s + sinSign * 0.5;
        
        if (radius < 0.13) {
            lFactor = 0;
            rFactor = 0;
        }

        Robot.driveTrain.setVelL(lFactor * radius * speed);
        Robot.driveTrain.setVelR(rFactor * radius * speed);

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
