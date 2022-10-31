package org.firstinspires.ftc.teamcode.backend.commandbased;

public interface Command {

    boolean isInterruptable();
    Subsystem[] subsystemsUsed();

    void init();

    void update();

    boolean isOver();

    void stop(boolean isInterrupted);

}
