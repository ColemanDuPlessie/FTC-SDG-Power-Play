package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class SequentialCommandGroup implements Command {

    private final ArrayList<Command> commands = new ArrayList<Command>();
    private int currentCommandIndex = -1;
    private Set<Subsystem> using = new HashSet<Subsystem>();
    private boolean isInterruptable = true;

    public SequentialCommandGroup(Command... commands) {
        addCommands(commands);
    }

    public void addCommands(Command... commands) {

        if (currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a CommandGroup while the group is running");
        }

        for (Command command : commands) {
            this.commands.add(command);
            using.addAll(Arrays.asList(command.subsystemsUsed()));
            isInterruptable &= command.isInterruptable();
        }
    }

    @Override
    public Subsystem[] subsystemsUsed() {
        Subsystem[] ans = new Subsystem[using.size()];
        using.toArray(ans);
        return ans;
    }

    public boolean isInterruptable() {return isInterruptable;}

    @Override
    public void init() {
        currentCommandIndex = 0;

        if (!commands.isEmpty()) {
            commands.get(0).init();
        }
    }

    @Override
    public void update() {
        if (commands.isEmpty()) {
            return;
        }

        Command currentCommand = commands.get(currentCommandIndex);

        currentCommand.update();
        if (currentCommand.isOver()) {
            currentCommand.stop(false);
            currentCommandIndex++;
            if (currentCommandIndex < commands.size()) {
                commands.get(currentCommandIndex).init();
            }
        }
    }

    @Override
    public void stop(boolean interrupted) {
        if (interrupted && !commands.isEmpty()) {
            commands.get(currentCommandIndex).stop(true);
        }
        currentCommandIndex = -1;
    }

    @Override
    public boolean isOver() {
        return currentCommandIndex == commands.size();
    }

}
