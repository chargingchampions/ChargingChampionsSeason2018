package org.usfirst.frc.team6560.robot.subsystems;

import org.usfirst.frc.team6560.robot.Robot;
import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.commands.ManualDrive;
import org.usfirst.frc.team6560.robot.util.DriveMotor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;




public class DriveTrainOne extends Subsystem {
  public static double RATIO = 373;
  private DriveMotor motorR1;
  private DriveMotor motorL1;
  private DriveMotor motorR2;
  private DriveMotor motorL2;
		
	public DriveTrainOne() {
		super();

    motorR1 = new DriveMotor(RobotMap.R1_MOTOR, true);
    motorR2 = new DriveMotor(RobotMap.R2_MOTOR, true);

    motorL1 = new DriveMotor(RobotMap.L1_MOTOR);
    motorL2 = new DriveMotor(RobotMap.L2_MOTOR);

  }

		
	@Override
	public void periodic() {
		super.periodic();

    System.out.println(motorL1.getPos());
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ManualDrive());
	}


    public void setVelL(double vel) {
      motorL1.setRPM(vel * RATIO);
      motorL2.setRPM(vel * RATIO);
  	}


    public void setVelR(double vel) {
      motorR1.setRPM(vel * RATIO);
      motorR2.setRPM(vel * RATIO);
    }
    
    public void stopImmediately() {
    	setVelL(0);
    	setVelR(0);
    }
    
    public double getEncoderPositionL() {
      return motorL1.getPos() / RATIO;

    }
    
    /**
     * 
     * @return encoder distance in feet
    //  */
    public double getEncoderPositionR() {
      return motorR1.getPos() / RATIO;
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
    return motorL1.getRPM() / RATIO;
	}
	
	/**
	 * 
	 * @return vel in ft/s
	 */
	public double getVelR() {
    return motorR1.getRPM() / RATIO;
	}
	
	
    
}

