package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;

public class DumpArm implements Command {

    private double priorPosition = 0.0;
    private double endArmTime;
    private double startSlidesTime;
    private ArmSubsystem arm;
    private ElapsedTime timer;
    private double pastTime;

    public DumpArm(ArmSubsystem system, ElapsedTime timer) {
        arm = system;
        this.timer = timer;
    }

    @Override
    public boolean isInterruptable() {
        return false;
    }

    @Override
    public Subsystem[] subsystemsUsed() {
        return new Subsystem[]{Subsystem.ARM, Subsystem.SLIDES};
    }

    @Override
    public void init() {
        priorPosition = arm.getPosition();
        arm.setTargetPosition(0.65);
        endArmTime = timer.milliseconds() + 750;
        startSlidesTime = endArmTime + 125;
        pastTime = timer.milliseconds();
    }

    @Override
    public void update() {
        if (!(pastTime > endArmTime) && timer.milliseconds() > endArmTime) {
            arm.setTargetPosition(0.1);
        }
        pastTime = timer.milliseconds();
    }

    @Override
    public boolean isOver() {
        return timer.milliseconds() >= startSlidesTime;
    }

    @Override
    public void stop(boolean isInterrupted) {
        arm.setTargetPosition(0.05);
    }
}
