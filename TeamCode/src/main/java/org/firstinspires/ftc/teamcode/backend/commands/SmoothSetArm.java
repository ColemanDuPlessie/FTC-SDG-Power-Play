package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;

public class SmoothSetArm extends CommandBase {

    private ElapsedTime timer;
    private long millis;
    private double targetPos;
    private double startHeight;
    private long startMillis;
    private ArmSubsystem arm;

    public SmoothSetArm(ArmSubsystem a, double targetPos, long millis, ElapsedTime timer) {
        arm = a;
        addRequirements(a);
        this.targetPos = targetPos;
        this.millis = millis;
        this.timer = timer;
    }

    @Override
    public void initialize() {
        this.startMillis = (long) timer.milliseconds();
        this.startHeight = arm.getPosition();
    }

    @Override
    public void execute() {
        arm.setTargetPosition(startHeight + (targetPos -startHeight)*((long)timer.milliseconds()-startMillis)/millis);
    }

    @Override
    public boolean isFinished() {return timer.milliseconds() >= millis + startMillis;}

    @Override
    public void end(boolean interrupted) {
    }
}
