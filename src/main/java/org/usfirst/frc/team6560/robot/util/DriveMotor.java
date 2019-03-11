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
    public DriveMotor(int id) {
        this(id, false);
    }
    public DriveMotor(int id, boolean inverted){
        this.id = id;
        motor = new CANSparkMax(id, MotorType.kBrushless);
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();
        motor.setInverted(inverted);
        motor.setClosedLoopRampRate(0);
        encoder.setPosition(0);

        pidController.setD(0);
        pidController.setP(0);
        pidController.setI(1E-6);
        pidController.setP(1E-5);
        pidController.setFF(0.0001855);
        pidController.setIZone(200);
        pidController.setIMaxAccum(0.05, 0);
    }

    public void stopImmediately() {
        setRPM(0);
        pidController.setIAccum(0);
    }

    public void setRPM(double input){
        pidController.setReference(input, ControlType.kVelocity);
        //SmartDashboard.putNumber(id + " target", input);
        //SmartDashboard.putNumber(id + " actual", getRPM());


    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public double getPos() {
        return encoder.getPosition();
    }
}
