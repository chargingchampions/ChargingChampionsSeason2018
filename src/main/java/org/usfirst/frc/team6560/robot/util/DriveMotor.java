package org.usfirst.frc.team6560.robot.util;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * Add your docs here.
 */
public class DriveMotor {
    private final int id;
    private CANSparkMax motor;
    private CANPIDController pidController;
    private CANEncoder encoder;
    private Mode mode;

    public DriveMotor(int id) {
        this(id, false);
    }

    public DriveMotor(int id, boolean inverted){
        this.id = id;
        motor = new CANSparkMax(id, MotorType.kBrushless);
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();
        motor.setInverted(inverted);
        encoder.setPosition(0);

        initVel();
    }

    public void stopImmediately() {
        if (mode == Mode.VEL) {
            setRPM(0);
            pidController.setIAccum(0);
        } else {
            setPos(getPos());
        }
        
    }

    public void setRPM(double input){
        initVel();
        pidController.setReference(input, ControlType.kSmartVelocity);
        //SmartDashboard.putNumber(id + " target", input);
        //SmartDashboard.putNumber(id + " actual", getRPM());
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public double getPos() {
        return encoder.getPosition();
    }

    public void setPos(double pos) {
        initPos();
        pidController.setReference(pos, ControlType.kSmartMotion);
    }

    private void initVel() {
        if  (mode != Mode.VEL) {
            motor.setClosedLoopRampRate(0);
            pidController.setD(0);
            pidController.setI(1E-6);
            pidController.setP(1E-5);
            pidController.setFF(0.0001855);
            pidController.setIZone(200);
            pidController.setOutputRange(-1.0, 1.0);
            pidController.setIMaxAccum(0.05, 0);
            pidController.setSmartMotionMaxAccel(3000, 0);

            mode = Mode.VEL;
        }
    }

    private void initPos() {
        if (mode != Mode.POS) {
            motor.setClosedLoopRampRate(0);
            pidController.setD(0);
            pidController.setI(0);
            pidController.setP(0.0003);
            pidController.setFF(0.0001855);
            pidController.setSmartMotionMaxVelocity(1200, 0);
            pidController.setSmartMotionMaxAccel(3000, 0);
            pidController.setSmartMotionAllowedClosedLoopError(0.002, 0);
            pidController.setOutputRange(-0.5, 0.5);

            mode = Mode.POS;
        }
    }

    private enum Mode {
        VEL,
        POS
    }
}
