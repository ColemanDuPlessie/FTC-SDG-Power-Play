package org.firstinspires.ftc.teamcode.backend.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

public class WaitMillis implements Command {

    private final ElapsedTime elapsedTime;

    private final double duration;

    private double endTime;

    public WaitMillis(ElapsedTime elapsedTime, double millis) {
        this.elapsedTime = elapsedTime;
        duration = millis;
    }


    @Override
    public boolean isInterruptable() { return false; }

    @Override
    public Subsystem[] subsystemsUsed() {
        return new Subsystem[0];
    }

    @Override
    public void init() {
        endTime = elapsedTime.milliseconds();
    }

    @Override
    public void update() {}

    @Override
    public boolean isOver() { return elapsedTime.milliseconds() >= endTime; }

    @Override
    public void stop(boolean isInterrupted) {}
}
