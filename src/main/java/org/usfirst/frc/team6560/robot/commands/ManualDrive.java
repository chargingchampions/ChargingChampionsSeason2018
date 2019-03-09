package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.subsystems.DriveTrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ManualDrive extends Command {
    public static final double ARCADE_TURN_SPEED = 0.9;
    public static final double JACK_TURN_SPEED = 5;
    
    public static final double MAX_SPEED = 15;
    
    private NetworkTable table;

    public ManualDrive() {
        requires(Robot.driveTrain);
        setInterruptible(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Running JoystickDrive command...");
        Robot.driveTrain.stop();
        table = Robot.nt.getTable("vision");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {    	
    	double x = -Robot.oi.logitech.getX();
        double y = -Robot.oi.logitech.getY();
        double multiplier = -(Robot.oi.logitech.getThrottle() - 1.0) / 2 * MAX_SPEED;
        
        double radius = Math.sqrt(x*x + y*y);
        double t = Math.atan2(y, x);



        double s = Math.min(JACK_TURN_SPEED / (2*multiplier), 0.5);

        if (s < 0) {
            s = 0;
        }
        
        double cosSign = Math.copySign(1.0, Math.cos(t));
        double sinSign = Math.copySign(1.0, Math.sin(t));
        double tanSign = Math.copySign(1.0, Math.tan(t));

        double funcVal = Math.cos(2*t);

        double lFactor = -cosSign * (s + tanSign * 0.5) * funcVal - cosSign * s + sinSign * 0.5;
        double rFactor = cosSign * (s - tanSign * 0.5) * funcVal + cosSign * s + sinSign * 0.5;

        double lTerm = 0;
        double rTerm = 0;
        
        if (Robot.oi.logitech.getRawButton(RobotMap.Logitech.BUTTON_10)) {
            double heading = table.getEntry("heading").getDouble(0);
            lTerm = limitMag(heading, 3);
            rTerm = limitMag(-heading, 3);

            System.out.println("heading: " + heading + " rTerm: " + rTerm + " lTerm: " + lTerm);

        }

        if (radius < 0.05) {
            lFactor = 0;
            rFactor = 0;
        }

        Robot.driveTrain.setVelL(lFactor * radius * multiplier + lTerm);
        Robot.driveTrain.setVelR(rFactor * radius * multiplier + rTerm);
    }

    private double limitMag(double input, double limit) {
        return Math.copySign(Math.min(Math.abs(input), limit), input);
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
