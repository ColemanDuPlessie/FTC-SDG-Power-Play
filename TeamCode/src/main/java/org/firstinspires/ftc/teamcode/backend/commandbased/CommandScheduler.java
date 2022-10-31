package org.firstinspires.ftc.teamcode.backend.commandbased;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.ConcurrentModificationException;
import java.util.HashMap;

public class CommandScheduler {
    private static final CommandScheduler INSTANCE = new CommandScheduler();
    public static CommandScheduler getInstance() {return INSTANCE;}

    private HashMap<Subsystem, Command> subsystemsInUse = new HashMap<Subsystem, Command>();
    private ArrayList<Command> activeCommands = new ArrayList<Command>();
    private HashMap<Subsystem, Command> defaultCommands = new HashMap<Subsystem, Command>();

    private CommandScheduler() {
        for (Subsystem system : Subsystem.values()) {
            subsystemsInUse.put(system, null);
            defaultCommands.put(system, null);
        }
    }

    public void telemetryDebug(Telemetry t) {
        t.addLine(activeCommands.toString());
    }

    public void reset() {
        for (Subsystem system : Subsystem.values()) {
            defaultCommands.put(system, null);
        }
    }

    public void update(Telemetry t) {
        ArrayList<Command> commandsToEnd = new ArrayList<Command>();
        for (int i = 0; i < activeCommands.size(); i++) {
            Command command = activeCommands.get(i);
            if (command.isOver()) {
                commandsToEnd.add(command);
            } else {
                command.update();
            }
        }
        for (Command ending : commandsToEnd) {
            endCommand(ending, false, false);
        }
    }

    void beginOpmode() {
        reset();
        for (Subsystem system : Subsystem.values()) {
            subsystemsInUse.put(system, null);
        }
        activeCommands = new ArrayList<Command>();
    }

    public void clearSubsystem(Subsystem system) {
        Command commandToEnd = subsystemsInUse.get(system);
        if (commandToEnd != null && commandToEnd.isInterruptable() && commandToEnd != defaultCommands.get(system)) {
            endCommand(commandToEnd, true, false);
        }
    }

    private void endCommand(Command command, boolean isInterrupted, boolean makesWayForOther) {
        command.stop(isInterrupted);
        activeCommands.remove(command);
        for (Subsystem system : command.subsystemsUsed()) {
            subsystemsInUse.put(system, null);
            if (defaultCommands.get(system) != null && !makesWayForOther && !activeCommands.contains(defaultCommands.get(system))) {
                scheduleCommand(defaultCommands.get(system));
            }
        }
    }

    public void scheduleCommand(Command toSchedule) {
        Subsystem[] subsystemsClaimed = toSchedule.subsystemsUsed();
        for (Subsystem system : subsystemsClaimed) {
            Command currentUser = subsystemsInUse.get(system);
            if (currentUser != null && !currentUser.isInterruptable()) { // If a subsystem is occupied by an uninterruptible command
                throw new SubsystemInUseException("The subsystem {} is already in use".format(system.toString()));
            }
        }
        // This point in the code will only be reached if all required subsystems are available
        for (Subsystem system : subsystemsClaimed) {
            Command currentUser = subsystemsInUse.get(system);
            if (currentUser != null) { // If we need to interrupt a command
                endCommand(currentUser, true, true);
            }
            subsystemsInUse.put(system, toSchedule);
        }
        activeCommands.add(toSchedule);
        toSchedule.init();
    }

    public void setDefaultCommand(Command command) {
        if (command.subsystemsUsed().length != 1) {
            throw new IllegalArgumentException("The specified command cannot be a default because it does not access exactly 1 subsystem.");
        }
        Subsystem system = command.subsystemsUsed()[0];
        defaultCommands.put(system, command);
        if (subsystemsInUse.get(system) == null) {scheduleCommand(command);}
    }

}
