package org.usfirst.frc.team6560.robot.util;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


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
    
    private CANSparkMax motor;
    private CANPIDController pidController;
    private CANEncoder encoder;

    public DriveMotor(int id){
        motor = new CANSparkMax(id, MotorType.kBrushless);
        pidController = motor.getPIDController();
        encoder = motor.getEncoder();

        encoder.setPosition(0);

        pidController.setD(0);
        pidController.setI(1E-6);
        pidController.setP(5E-5);
        pidController.setFF(0.0001735);
        pidController.setIZone(35);
        pidController.setIMaxAccum(0.01, 0);
    }

    public void setRPM(double input){
        pidController.setReference(input, ControlType.kVelocity);
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public double getPos() {
        return encoder.getPosition();
    }
}
