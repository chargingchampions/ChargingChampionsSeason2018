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
public class AutoVisionAlign extends Command {
    private NetworkTable table;
    private double heading = 0.0;
    private int stopCounter = 0;

    public AutoVisionAlign() {
        requires(Robot.driveTrain);
        setInterruptible(true);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        Robot.driveTrain.stop();
        table = Robot.nt.getTable("vision");
        heading = 0.0;
        stopCounter = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        heading = table.getEntry("heading").getDouble(0);
        SmartDashboard.putNumber("Heading", Math.round(heading * 100) / 100.0);

        if (Math.abs(Robot.driveTrain.getVelAngle()) <= 1)
        {
            stopCounter++;
            if (stopCounter >= 10) {
                Robot.driveTrain.setPosAngle(heading);
            }
        } else {
            stopCounter = 0;
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !Robot.oi.xboxDrive.getRawButton(RobotMap.XboxDrive.BUTTON_B);
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.driveTrain.stop();
        SmartDashboard.putNumber("Heading", Double.NaN);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
