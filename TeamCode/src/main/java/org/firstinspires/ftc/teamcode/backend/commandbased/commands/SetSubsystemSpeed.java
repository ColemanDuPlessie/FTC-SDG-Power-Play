package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SpeedControlled;

public class SetSubsystemSpeed implements Command {

    private final SpeedControlled system;
    private final Subsystem[] using = new Subsystem[1];
    private final double speed;

    public SetSubsystemSpeed(SpeedControlled system, Double speed, Subsystem using) {
        this.system = system;
        this.speed = (speed == null) ? 0.0 : speed;
        this.using[0] = using;
    }

    @Override
    public boolean isInterruptable() {return true;}

    @Override
    public Subsystem[] subsystemsUsed() {
        return using;
    }

    @Override
    public void init() {
        system.setSpeed(speed);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isOver() {return false;}

    @Override
    public void stop(boolean isInterrupted) {}
}
