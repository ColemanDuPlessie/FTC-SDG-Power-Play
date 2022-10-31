package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.backend.Robot19397;
import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;

public class DriveFromGamepad implements Command {

    private final Robot19397 robot;
    private final GamepadWrapper gamepad;
    private final boolean isFieldCentric;

    private final double topSpeed = 1.0;
    private final double defaultSpeed = 0.8;
    private final double minSpeed = 0.25;

    public DriveFromGamepad(Robot19397 robot, GamepadWrapper gamepad, boolean fieldCentric) {
        this.gamepad = gamepad;
        this.robot = robot;
        isFieldCentric = fieldCentric;
    }

    @Override
    public boolean isInterruptable() { return true; }

    @Override
    public Subsystem[] subsystemsUsed() {
        return new Subsystem[]{Subsystem.DRIVETRAIN};
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        double forward = -gamepad.getLeftStickY();
        double turn = gamepad.getRightStickX();
        double strafe = gamepad.getLeftStickX();
        double speed = defaultSpeed + gamepad.getRightTrigger() * (topSpeed-defaultSpeed) - gamepad.getLeftTrigger() * (defaultSpeed-minSpeed);
        if (isFieldCentric) {
            robot.driveFieldCentric(forward, turn, strafe, speed);
        } else {
            robot.driveSimple(forward, turn, strafe, speed);
        }
    }

    @Override
    public boolean isOver() { return false; }

    @Override
    public void stop(boolean isInterrupted) {}
}
