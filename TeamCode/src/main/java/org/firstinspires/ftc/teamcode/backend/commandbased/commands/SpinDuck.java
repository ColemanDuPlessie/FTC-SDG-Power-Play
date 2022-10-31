package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.backend.Robot19397;
import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;

@Config
public class SpinDuck implements Command {

    private final CarouselSubsystem carousel;
    private final boolean isBlue;
    private final ElapsedTime elapsedTime;

    public double slowSpeed = 0.7; // Assumes that we are on the Red Alliance
    public double slowDuration = 1000.0; // In milliseconds
    public static double endDuration = 700.0; // In milliseconds

    private double speedUpTime;
    private double endTime;
    private double lastTime;

    public SpinDuck(CarouselSubsystem carousel, boolean isBlueSide, ElapsedTime elapsedTime) {
        this.carousel = carousel;
        isBlue = isBlueSide;
        this.elapsedTime = elapsedTime;
    }

    public SpinDuck(CarouselSubsystem carousel, boolean isBlueSide, ElapsedTime elapsedTime, boolean isSlow) {
        this.carousel = carousel;
        isBlue = isBlueSide;
        this.elapsedTime = elapsedTime;
        if (isSlow) {
            slowSpeed /= 1.5;
            slowDuration *= 2.0;
        }
    }

    @Override
    public boolean isInterruptable() { return true; }

    @Override
    public Subsystem[] subsystemsUsed() {
        return new Subsystem[]{Subsystem.CAROUSEL};
    }

    @Override
    public void init() {
        lastTime = elapsedTime.milliseconds();
        speedUpTime = lastTime + slowDuration;
        endTime = speedUpTime + endDuration;
        carousel.setSpeed(isBlue ? -slowSpeed : slowSpeed);
    }

    @Override
    public void update() {
        double currTime = elapsedTime.milliseconds();
        if (currTime >= speedUpTime && lastTime < speedUpTime) {carousel.setSpeed(isBlue ? -1 : 1);}
        lastTime = currTime;
    }

    @Override
    public boolean isOver() { return lastTime >= endTime; }

    @Override
    public void stop(boolean isInterrupted) {
        carousel.setSpeed(0);
    }
}
