package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.PIDControlWrapper;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;

public class MakeIntakeVertical implements Command {

    private final static double kP = 0.00500;
    private final static double kI = 0.00000;
    private final static double kD = 0.00003;

    private final static double ticksPerRotation = 537.7;

    private int targetPosition;
    private PIDControlWrapper wrapper;

    private final IntakeSubsystem intake;
    private final ElapsedTime timer;

    public MakeIntakeVertical(IntakeSubsystem intake, ElapsedTime timer) {
        this.intake = intake;
        this.timer = timer;
    }

    @Override
    public boolean isInterruptable() {
        return true;
    }

    @Override
    public Subsystem[] subsystemsUsed() {
        return new Subsystem[]{Subsystem.INTAKE};
    }

    @Override
    public void init() {
        wrapper = new PIDControlWrapper(intake, new PIDController(kP, kI, kD, timer));
        double currPos = intake.getPosition();
        double currSpeed = intake.getSpeed();
        if (currSpeed >= 0) {
            targetPosition = (int)(Math.ceil(currPos / (ticksPerRotation/2.0)) * (ticksPerRotation/2.0));
        } else {
            targetPosition = (int)(Math.floor(currPos / (ticksPerRotation/2.0)) * (ticksPerRotation/2.0));
        }
        wrapper.setTargetPosition(targetPosition);
    }

    @Override
    public void update() {
        wrapper.update();
    }

    @Override
    public boolean isOver() {
        return false;
    }

    @Override
    public void stop(boolean isInterrupted) {}

}
