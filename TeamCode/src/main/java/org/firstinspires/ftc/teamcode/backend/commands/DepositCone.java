package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

public class DepositCone extends SequentialCommandGroup {

    public DepositCone(SlidesSubsystem s, ArmSubsystem a, DepositSubsystem d, double targetHeight) {
        addCommands(new InstantCommand(() -> prepare(s, a, d, targetHeight)),
                new WaitUntilCommand(() -> s.getPosition() > 0.8*targetHeight),
                new InstantCommand(() -> a.setTargetPosition(0.8)),
                new WaitUntilCommand(() -> a.getPosition() > 0.75),
                new InstantCommand(() -> d.deposit()),
                new WaitCommand(1000),
                new InstantCommand(() -> a.setTargetPosition(0.2)),
                new InstantCommand(() -> d.hold()),
                new WaitUntilCommand(() -> a.getPosition() < 0.5),
                new InstantCommand(() -> s.setTargetPosition(0.0)),
                new WaitUntilCommand(() -> s.getPosition() < 0.2),
                new InstantCommand(() -> a.setTargetPosition(0.0)));
        addRequirements(s);
        addRequirements(a);
        addRequirements(d);
    }

    private void prepare(SlidesSubsystem s, ArmSubsystem a, DepositSubsystem d, double targetHeight) {
        s.setTargetPosition(targetHeight);
        a.setTargetPosition(0.4);
        d.hold();
    }

}
