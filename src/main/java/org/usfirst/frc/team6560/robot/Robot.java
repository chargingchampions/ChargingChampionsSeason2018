/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6560.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import org.usfirst.frc.team6560.robot.subsystems.DriveTrain;
import org.usfirst.frc.team6560.robot.subsystems.Elevator;
import org.usfirst.frc.team6560.robot.subsystems.ElevatorPistons;
import org.usfirst.frc.team6560.robot.subsystems.Grabber;
import org.usfirst.frc.team6560.robot.subsystems.RearHatch;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI oi;
	
	public static DriveTrain driveTrain;
	public static Elevator elevator;
	public static Grabber grabber;
	public static RearHatch rearHatch;
	
	public static NetworkTableInstance nt;
	public static ElevatorPistons elevatorPistons;

	public DifferentialDrive motorDrive;

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
	//	CameraServer.getInstance().startAutomaticCapture();
	//	CameraServer.getInstance().startAutomaticCapture();
		driveTrain = new DriveTrain();
		elevator = new Elevator();
		grabber = new Grabber();
		rearHatch = new RearHatch();
		elevatorPistons = new ElevatorPistons();
		oi = new OI();
		nt = NetworkTableInstance.getDefault();
		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
		
	
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// WPI_TalonSRX	motorL1 = new WPI_TalonSRX(RobotMap.L1_MOTOR);
		// WPI_TalonSRX	motorL2 = new WPI_TalonSRX(RobotMap.L2_MOTOR);
		// WPI_TalonSRX	motorR1 = new WPI_TalonSRX(RobotMap.R1_MOTOR);
		// WPI_TalonSRX	motorR2 = new WPI_TalonSRX(RobotMap.R2_MOTOR);

		// initializeMotorManual(motorL1, 0.5);
		// initializeMotorManual(motorL2, 0.5);
		// initializeMotorManual(motorR1, 0.5);
		// initializeMotorManual(motorR2, 0.5);


 

		// SpeedControllerGroup motorGroupL = new SpeedControllerGroup(motorL1, motorL2);
		// SpeedControllerGroup motorGroupR = new SpeedControllerGroup(motorR1, motorR2);

		// motorDrive = new DifferentialDrive(motorGroupL,motorGroupR);

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}				
		//Scheduler.getInstance().add(new AutoStraightDistance(10));
	}

	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		
		double multiplier = -(Robot.oi.logitech.getThrottle() - 1.0) / 2;
		motorDrive.arcadeDrive(Robot.oi.logitech.getY() * multiplier, Robot.oi.logitech.getX()*0.7);

		 Scheduler.getInstance().run();
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public static void initializeMotorManual(WPI_TalonSRX motor) {
		initializeMotorManual(motor, 2);
	}
	
	public static void initializeMotorManual(WPI_TalonSRX motor, double ramp) {
		motor.config_kF(0, 0);
		motor.config_kP(0, 0);
		motor.config_kD(0, 0);
		motor.config_kI(0, 0);
	    	    
		motor.configOpenloopRamp(ramp, 100);
	}
}
