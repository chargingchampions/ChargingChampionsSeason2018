package org.usfirst.frc.team6560.robot.subsystems;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.commands.ManualDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {
	public static final double UNITS_PER_FOOT = 4096 / (Math.PI / 2.0);
	public static final double RAMP_TIME = 0.5;

	private WPI_TalonSRX motorR1;
	private WPI_TalonSRX motorR2;
	
	private WPI_TalonSRX motorL1;
	private WPI_TalonSRX motorL2;
	
	private double velL = 0.0;
	private double velR = 0.0;
	
	private boolean isSafe = true;
	private double kI_safety = 0.0008;
		
	public DriveTrain() {
		super();
		
		setManual();
	}
	
	public void setAutonomous()
	{
//		motorR1 = new WPI_TalonSRX(RobotMap.R1_ID);
//		motorR2 = new WPI_TalonSRX(RobotMap.R2_ID);
//
//	    motorL1 = new WPI_TalonSRX(RobotMap.L1_ID);
//	    motorL2 = new WPI_TalonSRX(RobotMap.L2_ID);
//	    
//	    motorL1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
//	    motorR1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
//
//	    motorL1.setSensorPhase(true);
//	    
//	    motorR1.setInverted(true);
//	    motorR2.setInverted(true);
//	    
//	    motorR1.setSensorPhase(true);
//	    
//	    motorR2.follow(motorR1);
//	    motorL2.follow(motorL1);
//	    
//		motorL1.config_kF(0, 0);
//	    motorR1.config_kF(0, 0);
//	    
//	    motorL1.config_kP(0, 0);
//	    motorR1.config_kP(0, 0);
//	   
//	    motorL1.config_kD(0, 0);
//	    motorR1.config_kD(0, 0);
//	    
//	    motorR1.config_kI(0, 0.0008);
//	    motorL1.config_kI(0, 0.0008);
//	    
//	    kI_safety = 0.0008;
//	    
//	    motorL1.configClosedloopRamp(0, 100);
//	    motorR1.configClosedloopRamp(0, 100);
	}
	
	public void setManual()
	{
		motorR1 = new WPI_TalonSRX(RobotMap.R1_MOTOR);
		motorR2 = new WPI_TalonSRX(RobotMap.R2_MOTOR);

	    motorL1 = new WPI_TalonSRX(RobotMap.L1_MOTOR);
	    motorL2 = new WPI_TalonSRX(RobotMap.L2_MOTOR);
	    
	    motorL1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
	    motorR1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);

	    motorL1.setSensorPhase(true);
	    
	    motorR1.setInverted(true);
	    motorR2.setInverted(true);
	    
	    motorR1.setSensorPhase(true);
	    
	    motorR2.follow(motorR1);
	    motorL2.follow(motorL1);
	    
	    motorL1.config_kF(0, 0.483307086);
		motorR1.config_kF(0, 0.483307086);
		    
		System.out.println("setting kP");

	    motorL1.config_kP(0, 0);
	    motorR1.config_kP(0, 0);
		   
		System.out.println("setting kD");

	    motorL1.config_kD(0, 0);
	    motorR1.config_kD(0, 0);
	    
		System.out.println("setting kI");

	    motorR1.config_kI(0, 0);
	    motorL1.config_kI(0, 0);
	    
	    kI_safety = 0;
	    
	    motorL1.configClosedloopRamp(RAMP_TIME, 100);
	    motorR1.configClosedloopRamp(RAMP_TIME, 100);
	    
	}
	
	private int safetyCounter = 0;
	
	@Override
	public void periodic() {
		super.periodic();
				
//		if (safetyCounter < 0) {
//			safetyCounter = 0;
//		}
//		
//		System.out.println(motorL1.getIntegralAccumulator() + ", " + motorR1.getIntegralAccumulator());
//		System.out.println(motorL1.getSelectedSensorVelocity() + ", " + motorR1.getSelectedSensorVelocity());
//
//		if (Math.abs(motorL1.getSelectedSensorVelocity(0)) < 5.0 && Math.abs(motorL1.getIntegralAccumulator()) * kI_safety > 104.0) {
//			safetyCounter += 3;
//		}
//		
//		if (Math.abs(motorR1.getSelectedSensorVelocity(0)) < 5.0 && Math.abs(motorR1.getIntegralAccumulator()) * kI_safety > 104.0) {
//			safetyCounter += 3;
//		}
// 

//		if (Math.abs(motorL1.getIntegralAccumulator()) > 20000000.0) {
//			isSafe = false;
//		}
//		
//		if (Math.abs(motorR1.getIntegralAccumulator()) > 20000000.0) {
//			isSafe = false;
//		}
		
//		--safetyCounter;
//		
//		if (safetyCounter > 45) {
//			isSafe = false;
//		}
		
		if (velL == 0 && velR == 0 && Math.abs(motorL1.getIntegralAccumulator()) < 130000  && Math.abs(motorR1.getIntegralAccumulator()) < 130000) {
    		motorL1.setIntegralAccumulator(motorL1.getIntegralAccumulator() * 0.9);
    		motorR1.setIntegralAccumulator(motorR1.getIntegralAccumulator() * 0.9);
    	}
		
		updateMotorControllers();
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	@Override
	public void initDefaultCommand() {
		// TODO re-enable JoystickDrive after re-implementing safety features
		
		setDefaultCommand(new ManualDrive());
	}

	public void updateMotorControllers() {
		if (isSafe) {
			motorL1.set(ControlMode.Velocity, velL);
	    	motorR1.set(ControlMode.Velocity, velR);
	    	
		} else {
			System.out.println("unsafe");
			motorL1.set(ControlMode.Velocity, 0); // Cannot call stopImmediately() because it will be recursive
	    	motorR1.set(ControlMode.Velocity, 0);
	    	
			motorL1.setIntegralAccumulator(0);
	    	motorR1.setIntegralAccumulator(0);
		}
		
	}

	/**
	 * 
	 * @param vel in ft/s
	 */
    public void setVelL(double vel) {
    	velL = vel*UNITS_PER_FOOT/10;
    	updateMotorControllers();
    }
    
    /**
     * 
     * @param vel in ft/s
     */
    public void setVelR(double vel) {
    	velR = vel*UNITS_PER_FOOT/10;
    	updateMotorControllers();
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
    public double getEncoderPositionL() {
    	return motorL1.getSelectedSensorPosition(0) /  UNITS_PER_FOOT;
    }
    
    /**
     * 
     * @return encoder distance in feet
     */
    public double getEncoderPositionR() {
    	return motorR1.getSelectedSensorPosition(0) /  UNITS_PER_FOOT;
    }
    
    public void stop() {
    	velL = 0.0;
    	velR = 0.0;
    	
    	updateMotorControllers();
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

