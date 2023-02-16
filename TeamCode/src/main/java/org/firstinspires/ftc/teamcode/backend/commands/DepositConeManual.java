package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

public class DepositConeManual extends SequentialCommandGroup {

    public DepositConeManual(SlidesSubsystem s, ArmSubsystem a, DepositSubsystem d, ElapsedTime timer) {
        addCommands(new InstantCommand(() -> d.deposit()),
                new WaitCommand(1000),
                new InstantCommand(() -> a.setTargetPosition(0.4)),
                new InstantCommand(() -> d.hold()),
                new WaitUntilCommand(() -> a.getPosition() < 0.5),
                new SmoothSetSlides(s, 0.0, 5000, timer)); // TODO: speed up?
        addRequirements(s);
        addRequirements(a);
        addRequirements(d);
    }

}
