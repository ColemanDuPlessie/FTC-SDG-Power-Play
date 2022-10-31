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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.backend.commandbased.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.commandbased.SubsystemInUseException;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.DriveFromGamepad;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.DumpArm;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.HoldSubsystemPosition;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.MakeIntakeVertical;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.RunRunnable;
import org.firstinspires.ftc.teamcode.backend.utilities.OverrideGamepadWrapper;
import org.firstinspires.ftc.teamcode.backend.utilities.RadioButtons;

import java.util.HashMap;
import java.util.function.Supplier;


/**
 * I should probably document this...
 */

@TeleOp(name="Outreach Demo")
public class DemoTeleop extends CommandbasedOpmode {

    @Override
    public void init() {

        robot.init(hardwareMap, true);
        pad1 = new OverrideGamepadWrapper(pad1, pad2, 0.35);

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

        try {
            scheduler.scheduleCommand(new DriveFromGamepad(robot, pad1, SetDrivingStyle.isFieldCentric));

        } catch (SubsystemInUseException e) {} // This catch block will never occur


        scheduler.setDefaultCommand(new HoldSubsystemPosition(robot.slides,
                new RadioButtons(new HashMap<Supplier<Boolean>, Object>() {{
                    put(pad1::getRightBumper, 0.1); // The slides must be down if we're running the intake
                }}, false), Subsystem.SLIDES, 0.1, () -> pad1.getX() ? 0.1 : null));

        scheduler.setDefaultCommand(new HoldSubsystemPosition(robot.arm,
                new RadioButtons(new HashMap<Supplier<Boolean>, Object>() {{
                    put(pad1::getX, 0.1);
                }}, false), Subsystem.ARM, 0.1));

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}