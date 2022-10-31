package org.firstinspires.ftc.teamcode.backend.utilities;

import com.qualcomm.robotcore.hardware.Gamepad;

public class OverrideGamepadWrapper extends GamepadWrapper{

    private final double speedMult;
    private final Gamepad override;

    public OverrideGamepadWrapper(GamepadWrapper overridden, GamepadWrapper overriding, double overriddenSlowdown) {
        super(overridden.g);
        this.speedMult = overriddenSlowdown;
        override = overriding.g;
    }

    public boolean getX() {return g.x && !override.a;}

    public double getLeftStickX() { return !override.a ? ((g.left_stick_button ? 0 : g.left_stick_x) * speedMult) : override.left_stick_x;}
    public double getLeftStickY() { return !override.a ? ((g.left_stick_button ? 0 : g.left_stick_y) * speedMult) : override.left_stick_y;}
    public double getAltLeftStickX() { return (g.left_stick_button ? g.left_stick_x : 0) * speedMult;}
    public double getAltLeftStickY() { return (g.left_stick_button ? g.left_stick_y : 0) * speedMult;}
    public double getRightStickX() { return !override.a ? ((g.right_stick_button ? 0 : g.right_stick_x) * speedMult) : override.right_stick_x;}
    public double getRightStickY() { return !override.a ? ((g.right_stick_button ? 0 : g.right_stick_y) * speedMult) : override.right_stick_y;}
    public double getAltRightStickX() { return (g.right_stick_button ? g.right_stick_x : 0) * speedMult;}
    public double getAltRightStickY() { return (g.right_stick_button ? g.right_stick_y : 0) * speedMult;}
}
