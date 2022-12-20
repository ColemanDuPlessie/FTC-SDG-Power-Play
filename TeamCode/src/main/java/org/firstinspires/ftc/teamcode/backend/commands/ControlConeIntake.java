package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.backend.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSlidesSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;

public class ControlConeIntake extends CommandBase {

    private final IntakeSubsystem cone;
    private final IntakeSlidesSubsystem slides;

    private double previousPos;

    public ControlConeIntake(IntakeSubsystem cone, IntakeSlidesSubsystem slides) {
        this.cone = cone;
        addRequirements(cone);
        this.slides = slides;
    }

    @Override
    public void initialize() {previousPos = slides.getTargetPosition();}

    @Override
    public void execute() {
        double currentPos = slides.getTargetPosition();
        if (currentPos > previousPos) {
            previousPos = currentPos;
            cone.lower();
        } else if (currentPos < previousPos) {
            previousPos = currentPos;
            cone.raise();
        }
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean isInterrupted) {}
}
