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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.backend.commandbased.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.commandbased.SubsystemInUseException;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.DriveFromGamepad;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.HoldSubsystemPosition;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.backend.utilities.PressableButton;
import org.firstinspires.ftc.teamcode.backend.utilities.RadioButtons;

import java.util.HashMap;
import java.util.function.Supplier;


/**
 * I should probably document this...
 */

@TeleOp(name="Simple Motor Tester")
public class MotorTester extends OpMode {

    private DcMotor testMotor;
    private double speed = 0;
    private double speedIncrement = 0.05;
    private PressableButton incBut;
    private PressableButton decBut;
    private GamepadWrapper pad1;

    public void incrementSpeed() {
        speed = Math.min(speed+speedIncrement, 1.0);
        testMotor.setPower(speed);
    }

    public void decrementSpeed() {
        speed = Math.max(speed-speedIncrement, -1.0);
        testMotor.setPower(speed);
    }

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        pad1 = new GamepadWrapper(gamepad1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        incBut = new PressableButton(pad1::getA, this::incrementSpeed);
        decBut = new PressableButton(pad1::getB, this::decrementSpeed);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        incBut.update();
        decBut.update();
        telemetry.addData("Speed", speed);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}