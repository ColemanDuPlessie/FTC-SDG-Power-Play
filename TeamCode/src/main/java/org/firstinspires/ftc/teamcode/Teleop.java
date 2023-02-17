/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.AutoTargetPole;
import org.firstinspires.ftc.teamcode.backend.commands.ContextSensitiveRetract;
import org.firstinspires.ftc.teamcode.backend.commands.DepositConeManual;
import org.firstinspires.ftc.teamcode.backend.commands.DriveFromGamepad;


/**
 * I should probably document this...
 */

@TeleOp(name="Teleop (THIS ONE)")
public class Teleop extends CommandbasedOpmode {

    @Override
    public void init() {
        robot.init(hardwareMap, true);
    }

    @Override
    public void start() {
        scheduler.setDefaultCommand(robot.drivetrain, new DriveFromGamepad(robot.drivetrain, pad1, SetDrivingStyle.isFieldCentric));

        GamepadEx gamepad = new GamepadEx(gamepad1);

/*        new Trigger(() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5)
                .whenInactive(() -> robot.intakeArm.pingpong());*/

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT) // TODO phase out this whole block of code
                .whenReleased(() -> robot.arm.incrementTargetPosition(-0.2)); // TODO closely inspect setpoints
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenReleased(() -> robot.arm.incrementTargetPosition(0.2));

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenReleased(() -> robot.arm.incrementTargetPosition(-0.2)); // TODO closely inspect setpoints
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenReleased(() -> robot.arm.incrementTargetPosition(0.2));

/*        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenReleased(() -> robot.intakeSlides.incrementTargetPosition(-0.2)); // TODO closely inspect setpoints
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenReleased(() -> {robot.intakeSlides.incrementTargetPosition(0.2); robot.intakeArm.extend();});*/

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenReleased(() -> robot.slides.incrementTargetPosition(0.3)); // TODO closely inspect setpoints
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenReleased(() -> robot.slides.incrementTargetPosition(-0.3));

        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenReleased(() -> robot.deposit.toggleIntake());
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenReleased(new DepositConeManual(robot.slides, robot.arm, robot.deposit, timer));
/*        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenReleased(new ContextSensitiveRetract(robot.intakeSlides, robot.intakeArm, robot.intake, robot.slides, robot.arm, robot.deposit, timer));
        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenReleased(() -> robot.intake.toggle());*/

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new AutoTargetPole(robot.drivetrain, robot.camera, timer, pad1, gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)), true);
    }

    @Override
    public void loop() {
        telemetry.addData("Arm pos", robot.arm.getPosition());
        telemetry.addData("Target pos", robot.arm.getTargetPosition());
        telemetry.addData("Arm spd", robot.arm.motor.getPower());
        telemetry.addLine();
        telemetry.addData("Slides pos", robot.slides.getPosition());
        telemetry.addData("Target pos", robot.slides.getTargetPosition());
        telemetry.addData("Slides spd", robot.slides.motor.getPower());
        telemetry.addData("Driving", scheduler.requiring(robot.drivetrain));
        AutoTargetPole vision = (AutoTargetPole)scheduler.requiring(robot.camera);
        if (vision != null) {
            telemetry.addLine();
            vision.debug(telemetry);
        }
    }
}