package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;

public class RunRunnableOnce implements Command {

    private final Runnable toRun;

    public RunRunnableOnce(Runnable r) {
        toRun = r;
    }

    @Override
    public boolean isInterruptable() {return true;}

    @Override
    public Subsystem[] subsystemsUsed() {return new Subsystem[0];}

    @Override
    public void init() {toRun.run();}

    @Override
    public void update() {}

    @Override
    public boolean isOver() {
        return true;
    }

    @Override
    public void stop(boolean isInterrupted) {}
}
