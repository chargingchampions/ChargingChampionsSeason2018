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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class DriveTrainOne extends Subsystem {
  public static double DISTANCE_RATIO = 6.047887837;
  public static double TIME_RATIO = 60.0;
  public static double ANGLE_RATIO = 360.0 / 5.011003875;

  public static double ACCELERATION = 14.0;

  private DriveMotor motorR1;
  private DriveMotor motorL1;
  private DriveMotor motorR2;
  private DriveMotor motorL2;

  private double velL = 0.0;
  private double velR = 0.0;

  private double targetVelL = 0.0;
  private double targetVelR = 0.0;

  private Mode mode;
		
	public DriveTrainOne() {
		super();

    motorR1 = new DriveMotor(RobotMap.R1_MOTOR, true);
    motorR2 = new DriveMotor(RobotMap.R2_MOTOR, true);

    motorL1 = new DriveMotor(RobotMap.L1_MOTOR);
    motorL2 = new DriveMotor(RobotMap.L2_MOTOR);

    initVel();
  }

		
	@Override
	public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("angle", getPosAngle());
    if (mode == Mode.VEL) {
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
  
      if (velL == 0) {
        if (Math.abs(motorL1.getRPM()) < 60) {
          motorL1.stopImmediately();
          motorL2.stopImmediately();
        }
      }
  
      if (velR == 0) {
        if (Math.abs(motorR1.getRPM()) < 60) {
          motorR1.stopImmediately();
          motorR2.stopImmediately();
        }
      }
    }

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
      initVel();

      targetVelL = vel;
    }

    public void setPosL(double pos) {
      initPos();

      motorL1.setPos(pos * DISTANCE_RATIO);
      motorL2.setPos(pos * DISTANCE_RATIO);
    }

    public void setPosR(double pos) {
      initPos();

      motorR1.setPos(pos * DISTANCE_RATIO);
      motorR2.setPos(pos * DISTANCE_RATIO);
    }

    public void setVelR(double vel) {
      initVel();

      targetVelR = vel;
    }
    
    public void stopImmediately() {
      if (mode == Mode.VEL) {
        setVelL(0);
        setVelR(0);

        velL = 0;
        velR = 0;
      } else {
        setPosL(getPosL());
        setPosR(getPosR());
      }

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
      if (mode == Mode.VEL) {
        setVelL(0);
        setVelR(0);
      } else {
        setPosL(getPosL());
        setPosR(getPosR());
      }
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
  
  private void initPos() {
    if (mode != Mode.POS) {
      mode = Mode.POS;
    }
  }

  private void initVel() {
    if (mode != Mode.VEL) {
      velL = getVelL();
      velR = getVelR();

      mode = Mode.VEL;
    }
    
  }
	
    private enum Mode {
      VEL,
      POS
    }
}

