package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.SlidesSubsystem;

public class SmoothSetSlides extends CommandBase {

    private ElapsedTime timer;
    private long millis;
    private double targetHeight;
    private double startHeight;
    private long startMillis;
    private SlidesSubsystem slides;

    public SmoothSetSlides(SlidesSubsystem s, double targetHeight, long millis, ElapsedTime timer) {
        slides = s;
        addRequirements(s);
        this.targetHeight = targetHeight;
        this.millis = millis;
        this.timer = timer;
    }

    @Override
    public void initialize() {
        this.startMillis = (long) timer.milliseconds();
        this.startHeight = slides.getPosition();
    }

    @Override
    public void execute() {
        slides.setTargetPosition(startHeight + (targetHeight-startHeight)*((long)timer.milliseconds()-startMillis)/millis);
    }

    @Override
    public boolean isFinished() {return timer.milliseconds() >= millis + startMillis;}

    @Override
    public void end(boolean interrupted) {
    }
}
