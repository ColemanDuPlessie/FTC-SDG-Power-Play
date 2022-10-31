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

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.HoldSubsystemPosition;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.HoldSubsystemSpeed;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.MakeIntakeVertical;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.RunRunnable;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.RunRunnableOnce;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.SpinDuck;
import org.firstinspires.ftc.teamcode.backend.commandbased.commands.WaitMillis;
import org.firstinspires.ftc.teamcode.backend.cv.TeamShippingElementDetector;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.backend.utilities.ToggleButton;
import org.openftc.easyopencv.OpenCvCameraException;


/**
 * I should probably document this...
 */

@Autonomous(name="Auto (THIS ONE)")
@Config
public class Auto extends CommandbasedOpmode {

    SampleMecanumDrive drive;
    TrajectorySequence L;
    TrajectorySequence C;
    TrajectorySequence R;
    TeamShippingElementDetector tagDetector;
    TeamShippingElementDetector.POSITIONS tagPosition = null;

    public double STARTX = 36;
    public double STARTY = 63;
    public double STARTTHETA = 90;
    public double MIDX = 36;
    public double MIDY = 36;
    public double DRIFTX = -24;

    private int tagDetectionFails = 0;

    double startHeading;

    @Override
    public void init() {

        robot.init(hardwareMap, false);

        startHeading = robot.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, Math.toRadians(STARTTHETA));

        tagDetector = new TeamShippingElementDetector(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        L = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineTo(new Vector2d(MIDX-DRIFTX, STARTY))
                .lineTo(new Vector2d(MIDX-DRIFTX, MIDY))
                .build();

        C = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineTo(new Vector2d(MIDX, MIDY))
                .build();

        R = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .lineTo(new Vector2d(MIDX+DRIFTX, STARTY))
                .lineTo(new Vector2d(MIDX+DRIFTX, MIDY))
                .build();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        TeamShippingElementDetector.POSITIONS guess = tagDetector.getTagPosition();

        if (guess == null) {
            tagDetectionFails += 1;
        }
        if (tagDetectionFails > 20 || guess != null) {
            tagDetectionFails = 0;
            tagPosition = guess;
        }

        if (tagPosition != null) {
            telemetry.addLine("Tag is in the {} position".format(tagPosition.name()));
        } else { telemetry.addLine("Tag is not detected"); }
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        try {
            tagDetector.close();
        } catch (OpenCvCameraException e) {
            telemetry.addLine("Camera was not initialized. CV pipeline replaced by default behavior.");
        }
        if (tagPosition == TeamShippingElementDetector.POSITIONS.ONE) {
            scheduler.scheduleCommand(new FollowRRTraj(drive, L));
        } else if (tagPosition == TeamShippingElementDetector.POSITIONS.TWO || tagPosition == null) {
            scheduler.scheduleCommand(new FollowRRTraj(drive, C));
        } else if (tagPosition == TeamShippingElementDetector.POSITIONS.THREE) {
            scheduler.scheduleCommand(new FollowRRTraj(drive, R));
        }
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
        AutoToTeleopContainer.getInstance().setAngleDelta(startHeading-robot.getHeading()+Math.toRadians(180));
    }
}