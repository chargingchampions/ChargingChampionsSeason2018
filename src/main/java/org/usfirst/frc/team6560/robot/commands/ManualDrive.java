package org.usfirst.frc.team6560.robot.commands;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.subsystems.DriveTrain;
import org.usfirst.frc.team6560.robot.subsystems.DriveTrainOne;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ManualDrive extends Command {
    public static final double JACK_TURN_SPEED = 4.2;
    
    public static final int MAX_SPEED = 12;
    
    private NetworkTable table;

    private int speed = 3;
    private int lastPOV;

    private double heading = 0.0;

    public ManualDrive() {
        requires(Robot.driveTrain);
        setInterruptible(true);
        SmartDashboard.putNumber("Speed", speed);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Running JoystickDrive command...");
        Robot.driveTrain.stop();
        table = Robot.nt.getTable("vision");

        speed = 3;
        SmartDashboard.putNumber("Speed", speed);

        lastPOV = 0;
        
        resetVision();
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
        heading = table.getEntry("heading").getDouble(0);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Heading", Math.round(heading * 100) / 100.0);

        if (Robot.oi.xboxDrive.getRawButton(RobotMap.XboxDrive.BUTTON_B)) {
           executeVision();
        } else {
            resetVision();
            executeDrive();
        }


    }

    private void executeDrive() {
        double x = 0;
        double y = 0;

        x = -Robot.oi.xboxDrive.getRawAxis(RobotMap.XboxDrive.LEFT_JOY_X);
        y = -Robot.oi.xboxDrive.getRawAxis(RobotMap.XboxDrive.RIGHT_JOY_Y);
    	        
        double radius = Math.sqrt(x*x + y*y);
        double t = Math.atan2(y, x);

        double s = Math.min(JACK_TURN_SPEED / (2.0*speed), 0.5);

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


    private enum VisionStage {
        STOPPING,
        ALIGNING,
        MOVING
    }

    private int stopCounter;
    private VisionStage visionStage;

    private void resetVision() {
        stopCounter = 0;
        visionStage = VisionStage.STOPPING;
    }

    private void executeVision() {
        switch (visionStage) {
            case STOPPING: {
                Robot.driveTrain.stop();
                if (Math.abs(Robot.driveTrain.getVelL()) < 0.01 && Math.abs(Robot.driveTrain.getVelR()) < 0.01) {
                    visionStage = VisionStage.ALIGNING;
                }
                break;
            }
            case ALIGNING: {
                if (Math.abs(Robot.driveTrain.getVelAngle()) <= 1)
                {
                    stopCounter++;
                    if (stopCounter >= 10) {
                        if (heading > 0.5) {
                            setHeadingOutput(heading, 0);
                        } else {
                            visionStage = VisionStage.MOVING;
                        }
                    }
                } else {
                    stopCounter = 0;
                }

                break;
            }
            case MOVING: {
               setHeadingOutput(heading, -Robot.oi.xboxDrive.getRawAxis(RobotMap.XboxDrive.RIGHT_JOY_Y));
            }
        }
        
    }

    private void setHeadingOutput(double heading, double vel) {
        Robot.driveTrain.setPosL(Robot.driveTrain.getPosL() + heading * DriveTrainOne.ANGLE_RATIO);
        Robot.driveTrain.setPosR(Robot.driveTrain.getPosR() - heading * DriveTrainOne.ANGLE_RATIO);
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
