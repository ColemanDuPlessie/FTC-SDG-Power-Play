package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DepositSubsystem;

public class IntakeConeAuto extends SequentialCommandGroup {

    private static final double[] POSITIONS = {1.0, 0.96, 0.92, 0.88, 0.84};

    public IntakeConeAuto(ArmSubsystem a, DepositSubsystem d, int coneHeight, ElapsedTime timer) {
        addCommands(
                new InstantCommand(() -> a.setTargetPosition(0.6)),
                new WaitUntilCommand(() -> a.getPosition() > 0.5),
                new InstantCommand(d::intake),
                new WaitCommand(500),
                new SmoothSetArm(a, POSITIONS[coneHeight-1], 1500, timer),
                new ParallelCommandGroup(
                        new SmoothSetArm(a, 0.4, 500, timer),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new InstantCommand(d::hold)
                        )
                )
        );
        addRequirements(a);
        addRequirements(d);
    }

    public IntakeConeAuto(ArmSubsystem a, DepositSubsystem d, int coneHeight, ElapsedTime timer, Command next) {
        addCommands(
                new InstantCommand(() -> a.setTargetPosition(0.6)),
                new WaitUntilCommand(() -> a.getPosition() > 0.5),
                new InstantCommand(d::intake),
                new WaitCommand(500),
                new SmoothSetArm(a, POSITIONS[coneHeight-1], 1500, timer),
                new ParallelCommandGroup(
                        new SmoothSetArm(a, 0.4, 500, timer),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new ParallelCommandGroup(
                                        new ScheduleCommand(next),
                                        new InstantCommand(d::hold)
                                )
                        )
                )
        );
        addRequirements(a);
        addRequirements(d);
    }

}
