package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;

public class RunRunnable implements Command {

    private final Runnable toRun;

    public RunRunnable(Runnable r) {
        toRun = r;
    }

    @Override
    public boolean isInterruptable() {return true;}

    @Override
    public Subsystem[] subsystemsUsed() {return new Subsystem[0];}

    @Override
    public void init() {}

    @Override
    public void update() {toRun.run();}

    @Override
    public boolean isOver() {
        return false;
    }

    @Override
    public void stop(boolean isInterrupted) {}
}
