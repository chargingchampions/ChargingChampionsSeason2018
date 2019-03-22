package org.usfirst.frc.team6560.robot.subsystems;

import org.usfirst.frc.team6560.robot.RobotMap;
import org.usfirst.frc.team6560.robot.commands.ManualDrive;
import org.usfirst.frc.team6560.robot.util.DriveMotor;

import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrainOne extends Subsystem {
  public static double DISTANCE_RATIO = 6.047887837;
  public static double TIME_RATIO = 60.0;
  public static double ANGLE_RATIO = 360.0 / 5.011003875;

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
	}

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ManualDrive());
	}

    public void setVelAngle(double angle) {
      setVelL(angle / ANGLE_RATIO);
      setVelR(-angle / ANGLE_RATIO);
    }

    public double getVelAngle() {
      return ((getVelL() - getVelR()) / 2.0) * ANGLE_RATIO;

    }

    public void setPosAngle(double angle) {
      setPosL(getPosL() + angle / ANGLE_RATIO);
      setPosR(getPosR() - angle / ANGLE_RATIO);
    }

    public double getPosAngle() {
      return ((getPosL() - getPosR()) / 2.0) * ANGLE_RATIO;
    }


    public void setVelL(double vel) {
      motorL1.setRPM(vel * DISTANCE_RATIO * TIME_RATIO);
      motorL2.setRPM(vel * DISTANCE_RATIO * TIME_RATIO);
    }

    public void setVelR(double vel) {
      motorR1.setRPM(vel * DISTANCE_RATIO * TIME_RATIO);
      motorR2.setRPM(vel * DISTANCE_RATIO * TIME_RATIO);
    }

    public void setPosL(double pos) {
      motorL1.setPos(pos * DISTANCE_RATIO);
      motorL2.setPos(pos * DISTANCE_RATIO);
    }

    public void setPosR(double pos) {
      motorR1.setPos(pos * DISTANCE_RATIO);
      motorR2.setPos(pos * DISTANCE_RATIO);
    }
    
    public void stopImmediately() {
      setVelL(0);
      setVelR(0);

      motorL1.stopImmediately();
      motorR1.stopImmediately();
      motorL2.stopImmediately();
      motorR2.stopImmediately();
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

