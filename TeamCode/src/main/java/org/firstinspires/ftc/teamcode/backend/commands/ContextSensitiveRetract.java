package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

public class ContextSensitiveRetract extends SequentialCommandGroup {

    public ContextSensitiveRetract(IntakeSlidesSubsystem s, IntakeArmSubsystem a, IntakeSubsystem i,SlidesSubsystem s2, ArmSubsystem a2, DepositSubsystem d, ElapsedTime timer) {
        if (i.getPosition() == i.closedPosition) {
            addCommands(new InstantCommand(() -> {i.close(); a2.setTargetPosition(0.6); d.deposit();}),
                    new WaitCommand(250),
                    new InstantCommand(() -> {a.hide(); d.hold();}),
                    new ParallelCommandGroup(new SmoothSetIntakeSlides(s, 0.0, (int)(s.getPosition()*4000), timer),
                            new SmoothSetSlides(s2, 0.0, (int)(s2.getPosition()*4000), timer)),// TODO: speed up?
                    new InstantCommand(() -> a.retract()),
                    new WaitCommand(500),
                    new InstantCommand(() -> i.fullyOpen()),
                    new WaitCommand(250),
                    new InstantCommand(() -> a.hide()),
                    new WaitCommand(500),
                    new InstantCommand(() -> {a2.setTargetPosition(0.0); d.intake();}),
                    new WaitCommand(1000),
                    new InstantCommand(() -> {a2.setTargetPosition(0.6); d.hold();}));
            addRequirements(s2, a2, d);
        } else {
            addCommands(new InstantCommand(() -> i.open()),
                    new InstantCommand(() -> a.hide()),
                    new SmoothSetIntakeSlides(s, 0.0, (int)(s.getPosition()*4000), timer)); // TODO: speed up?
        }
        addRequirements(s, a, i);
    }

}
