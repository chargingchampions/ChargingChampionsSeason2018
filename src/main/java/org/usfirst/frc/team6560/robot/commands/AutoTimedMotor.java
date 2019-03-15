package org.usfirst.frc.team6560.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class AutoTimedMotor extends Command {
    private final Runnable start;
    private final Runnable end;
    private final int delay;

    private int counter;

    public AutoTimedMotor(Runnable start, Runnable end, int delay) {
        this.start = start;
        this.end = end;
        this.delay = delay;
    }

    @Override
    protected void initialize() {
        super.initialize();

        start.run();
        counter = 0;
    }

    @Override
    protected void execute() {
        super.execute();
        ++counter;
    }

    @Override
    protected boolean isFinished() {
        return counter >= delay;
    }

    @Override
    protected void end() {
        super.end();

        end.run();
    }
}