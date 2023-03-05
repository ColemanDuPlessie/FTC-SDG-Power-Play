package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

public class DepositConeAuto extends SequentialCommandGroup {

    public DepositConeAuto(SlidesSubsystem s, ArmSubsystem a, DepositSubsystem d, double targetHeight, ElapsedTime timer) {
        addCommands(new InstantCommand(() -> prepare(a, d, targetHeight)),
                new SmoothSetSlides(s, targetHeight, (long)(targetHeight*2500), timer),
                new InstantCommand(() -> a.setTargetPosition(0.7)),
                new WaitUntilCommand(() -> a.getPosition() > 0.65),
                new InstantCommand(() -> d.deposit()),
                new WaitCommand(500),
                new InstantCommand(() -> a.setTargetPosition(0.2)),
                new InstantCommand(() -> d.hold()),
                new WaitUntilCommand(() -> a.getPosition() < 0.5),
                new SmoothSetSlides(s, 0.0, (long)(targetHeight*2500), timer));
        addRequirements(s);
        addRequirements(a);
        addRequirements(d);
    }

    public DepositConeAuto(SlidesSubsystem s, ArmSubsystem a, DepositSubsystem d, double targetHeight, ElapsedTime timer, Command next) {
        addCommands(new InstantCommand(() -> prepare(a, d, targetHeight)),
                new SmoothSetSlides(s, targetHeight, (long)(targetHeight*2500), timer),
                new InstantCommand(() -> a.setTargetPosition(0.7)),
                new WaitUntilCommand(() -> a.getPosition() > 0.65),
                new InstantCommand(() -> d.deposit()),
                new WaitCommand(500),
                new InstantCommand(() -> a.setTargetPosition(0.2)),
                new InstantCommand(() -> d.hold()),
                new WaitUntilCommand(() -> a.getPosition() < 0.5),
                new ScheduleCommand(next),
                new SmoothSetSlides(s, 0.0, (long)(targetHeight*2500), timer));
        addRequirements(s);
        addRequirements(a);
        addRequirements(d);
    }

    private void prepare(ArmSubsystem a, DepositSubsystem d, double targetHeight) {
        a.setTargetPosition(0.4);
        d.hold();
    }

}
