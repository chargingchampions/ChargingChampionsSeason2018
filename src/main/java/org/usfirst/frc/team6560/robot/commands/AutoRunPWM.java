package org.usfirst.frc.team6560.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.command.Command;

public class AutoRunPWM extends Command {
    private final PWM pwm;
    private final double output;
    private final int frames;

    private int counter;

    public AutoRunPWM(int channel, double output, int frames) {
        this.pwm = new PWM(channel);
        this.output = output;
        this.frames = frames;
    }

    @Override
    protected void initialize() {
        super.initialize();
        
        counter = 0;
        pwm.setSpeed(output);
    }

    @Override
    protected void execute() {
        super.execute();
        ++counter;
    }

    @Override
    protected boolean isFinished() {
        return counter >= frames;
    }

    @Override
    protected void end() {
        pwm.setSpeed(0);
    }

    @Override
    protected void interrupted() {
        end();
    }


}