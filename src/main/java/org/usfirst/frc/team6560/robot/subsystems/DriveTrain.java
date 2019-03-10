package org.usfirst.frc.team6560.robot.subsystems;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.commands.ManualDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class DriveTrain extends Subsystem {
	public static final double UNITS_PER_FOOT = 4096 / (Math.PI / 2.0);
	public static final double RAMP_TIME = 0.5;

	public DifferentialDrive motorDrive;

	private WPI_TalonSRX motorR1;
	private WPI_TalonSRX motorR2;
	
	private WPI_TalonSRX motorL1;
	private WPI_TalonSRX motorL2;
		
	public DriveTrain() {
		super();
		setManual();
	}
	
	public void setManual()
	{
		motorR1 = new WPI_TalonSRX(RobotMap.R1_MOTOR);
		motorR2 = new WPI_TalonSRX(RobotMap.R2_MOTOR);

	    motorL1 = new WPI_TalonSRX(RobotMap.L1_MOTOR);
	    motorL2 = new WPI_TalonSRX(RobotMap.L2_MOTOR);
	    
	    motorL1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
	    motorR1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

	    motorR1.setInverted(true);
		motorR2.setInverted(true);
		
		motorL1.setInverted(false);
		motorL2.setInverted(false);
		
		motorL1.setSensorPhase(true);
	    motorR1.setSensorPhase(true);
	    //0.483307086
	    motorL1.config_kF(0, 0.483307086);
		motorR1.config_kF(0, 0.483307086);
			
		//The Kp = 0.0731
		//The Kf settles with 1400 error so the extra 10% throttle will be (0.1x1023) / 1400 = 0.0731
	    motorL1.config_kP(0, 0.0731);
	    motorR1.config_kP(0, 0.0731);
		   
	    motorL1.config_kD(0, 0);
	    motorR1.config_kD(0, 0);
	    
	    motorR1.config_kI(0, 0);
	    motorL1.config_kI(0, 0);
	    	    
	    motorL1.configClosedloopRamp(RAMP_TIME, 100);
		motorR1.configClosedloopRamp(RAMP_TIME, 100);

		motorR2.follow(motorR1);
		motorL2.follow(motorL2);

		// motorL2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
	    // motorR2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		
		// motorL2.setSensorPhase(true);
	    // motorR2.setSensorPhase(true);
	    
	    // motorL2.config_kF(0, 0.483307086);
		// motorR2.config_kF(0, 0.483307086);
			
		// //The Kp = 0.0731
		// //The Kf settles with 1400 error so the extra 10% throttle will be (0.1x1023) / 1400 = 0.0731
	    // motorL2.config_kP(0, 0);
	    // motorR2.config_kP(0, 0);
		   
	    // motorL2.config_kD(0, 0);
	    // motorR2.config_kD(0, 0);
	    
	    // motorR2.config_kI(0, 0);
	    // motorL2.config_kI(0, 0);
	    	    
	    // motorL2.configClosedloopRamp(RAMP_TIME, 100);
		// motorR2.configClosedloopRamp(RAMP_TIME, 100);


		//Second Version
		SpeedControllerGroup motorGroupL = new SpeedControllerGroup(motorL1, motorL2);
		SpeedControllerGroup motorGroupR = new SpeedControllerGroup(motorR1, motorR2);

		motorDrive = new DifferentialDrive(motorGroupL,motorGroupR);
	}
		
	@Override
	public void periodic() {
		super.periodic();

	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ManualDrive());
	}

	/**
	 * 
	 * @param vel in ft/s
	 */
    public void setVelL(double vel) {
		motorL1.set(ControlMode.Velocity, vel*UNITS_PER_FOOT/10);
		motorL2.set(ControlMode.Velocity, vel*UNITS_PER_FOOT/10);

	}
    
    /**
     * 
     * @param vel in ft/s
     */
    public void setVelR(double vel) {
		motorR1.set(ControlMode.Velocity, vel*UNITS_PER_FOOT/10);
		motorR2.set(ControlMode.Velocity, vel*UNITS_PER_FOOT/10);

	}
    
    public void stopImmediately() {
    	setVelL(0);
    	setVelR(0);
    	
    	motorL1.setIntegralAccumulator(0);
    	motorR1.setIntegralAccumulator(0);
    }
    
    /**
     * 
     * @return encoder distance in feet
     */
    public double getPosL() {
    	return motorL1.getSelectedSensorPosition(0) /  UNITS_PER_FOOT;
    }
    
    /**
     * 
     * @return encoder distance in feet
     */
    public double getPosR() {
    	return motorR1.getSelectedSensorPosition(0) /  UNITS_PER_FOOT;
    }
    
    public void stop() {
    	setVelL(0);
    	setVelR(0);
    }

    /**
     * 
     * @return vel in ft/s
     */
	public double getVelL() {
		return motorL1.getSelectedSensorVelocity(0) * 10 / UNITS_PER_FOOT;
	}
	
	/**
	 * 
	 * @return vel in ft/s
	 */
	public double getVelR() {
		return motorR1.getSelectedSensorVelocity(0) * 10 / UNITS_PER_FOOT;
	}
	
	
    
}

