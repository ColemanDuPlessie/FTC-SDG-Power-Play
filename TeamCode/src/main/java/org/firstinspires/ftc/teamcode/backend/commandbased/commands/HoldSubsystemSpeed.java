package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SpeedControlled;

import java.util.function.Supplier;

public class HoldSubsystemSpeed implements Command {

    private final SpeedControlled system;
    private final Subsystem[] using = new Subsystem[1];
    private final Supplier<Double> speedGetter;
    private double currSpeed = 0;

    public HoldSubsystemSpeed(SpeedControlled system, Supplier<Double> speedGetter, Subsystem using) {
        this.system = system;
        this.speedGetter = speedGetter;
        this.using[0] = using;
    }

    @Override
    public boolean isInterruptable() {return true;}

    @Override
    public Subsystem[] subsystemsUsed() {
        return using;
    }

    private void setSpeed() {
        Double speed = speedGetter.get();
        if (speed == null) {speed = 0.0;}
        if (speed == currSpeed) {return;}
        currSpeed = speed;
        system.setSpeed(speed);
    }

    @Override
    public void init() {
        this.setSpeed();
    }

    @Override
    public void update() {
        this.setSpeed();
    }

    @Override
    public boolean isOver() {return false;}

    @Override
    public void stop(boolean isInterrupted) {}
}
