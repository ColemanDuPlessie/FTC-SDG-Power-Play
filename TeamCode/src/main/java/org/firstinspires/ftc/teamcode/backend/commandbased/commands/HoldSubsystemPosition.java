package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.PositionControlled;
import org.firstinspires.ftc.teamcode.backend.subsystems.SpeedControlled;

import java.util.function.Supplier;

public class HoldSubsystemPosition implements Command {

    private final PositionControlled system;
    private final Subsystem[] using = new Subsystem[1];
    private final Supplier<Double> positionGetter;
    private final Supplier<Double> pausedPositionGetter;
    private double currPosition = -1.0;
    private final double defaultPosition;

    private boolean resumingFromInterrupt = false;

    public HoldSubsystemPosition(PositionControlled system, Supplier<Double> positionGetter, Subsystem using, double defaultPosition) {
        this.system = system;
        this.positionGetter = positionGetter;
        this.using[0] = using;
        this.defaultPosition = defaultPosition;
        pausedPositionGetter = () -> null;
    }

    public HoldSubsystemPosition(PositionControlled system, Supplier<Double> positionGetter, Subsystem using, double defaultPosition, Supplier<Double> pausedPositionGetter) {
        this.system = system;
        this.positionGetter = positionGetter;
        this.using[0] = using;
        this.defaultPosition = defaultPosition;
        this.pausedPositionGetter = pausedPositionGetter;
    }

    @Override
    public boolean isInterruptable() {return true;}

    @Override
    public Subsystem[] subsystemsUsed() {
        return using;
    }

    private void setPosition() {
        Double position = positionGetter.get();
        if (position == null) {position = currPosition;}
        if (position == currPosition) {return;}
        currPosition = position;
        system.setTargetPosition(position);
    }

    @Override
    public void init() {
        if (resumingFromInterrupt) {
            system.setTargetPosition(currPosition);
            resumingFromInterrupt = false;
        } else {
            this.setPosition();
        }
    }

    @Override
    public void update() {
        this.setPosition();
    }

    @Override
    public boolean isOver() {return false;}

    @Override
    public void stop(boolean isInterrupted) {
        if (isInterrupted) {
            // This is done so that, if a button is mapped to both end this command and trigger
            // a state change within it, the state change happens when the command resumes (if ever)
            Double position = positionGetter.get();
            Double pausedPosition = pausedPositionGetter.get();
            if (pausedPosition != null) {position = pausedPosition;}
            if (position != null) {currPosition = position;}
            resumingFromInterrupt = true;
        } else {
            system.setTargetPosition(defaultPosition);
        }
    }
}
