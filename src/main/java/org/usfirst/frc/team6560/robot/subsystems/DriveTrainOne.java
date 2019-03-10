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
  public static double DISTANCE_RATIO = 6.047887837;
  public static double TIME_RATIO = 60.0;
  public static double ACCELERATION = 14.0;

  private DriveMotor motorR1;
  private DriveMotor motorL1;
  private DriveMotor motorR2;
  private DriveMotor motorL2;

  private double velL = 0.0;
  private double velR = 0.0;

  private double targetVelL = 0.0;
  private double targetVelR = 0.0;
		
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

    if (targetVelL > velL) {
      velL += Math.min(ACCELERATION / 60, Math.abs(targetVelL - velL));
    } else {
      velL -= Math.min(ACCELERATION / 60, Math.abs(targetVelL - velL));
    }

    if (targetVelR > velR) {
      velR += Math.min(ACCELERATION / 60, Math.abs(targetVelR - velR));
    } else {
      velR -= Math.min(ACCELERATION / 60, Math.abs(targetVelR - velR));
    }

    motorR1.setRPM(velR * DISTANCE_RATIO * TIME_RATIO);
    motorR2.setRPM(velR * DISTANCE_RATIO * TIME_RATIO);

    motorL1.setRPM(velL * DISTANCE_RATIO * TIME_RATIO);
    motorL2.setRPM(velL * DISTANCE_RATIO * TIME_RATIO);

	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ManualDrive());
	}


    public void setVelL(double vel) {
      targetVelL = vel;
        	}


    public void setVelR(double vel) {
      targetVelR = vel;
    }
    
    public void stopImmediately() {
    	setVelL(0);
    	setVelR(0);
    }
    
    public double getPosL() {
      return motorL1.getPos() / DISTANCE_RATIO;

    }
    
    /**
     * 
     * @return encoder distance in feet
    //  */
    public double getPosR() {
      return motorR1.getPos() / DISTANCE_RATIO;
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
    return motorL1.getRPM() / (DISTANCE_RATIO * TIME_RATIO);
	}
	
	/**
	 * 
	 * @return vel in ft/s
	 */
	public double getVelR() {
    return motorR1.getRPM() / (DISTANCE_RATIO * TIME_RATIO);
	}
	
	
    
}

