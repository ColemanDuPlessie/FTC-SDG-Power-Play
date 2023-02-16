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
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.DepositConeAuto;
import org.firstinspires.ftc.teamcode.backend.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.cv.TeamShippingElementDetector;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCameraException;


/**
 * I should probably document this...
 */

@Autonomous(name="Auto (THIS ONE)")
@Config
public class Auto extends CommandbasedOpmode {

    SampleMecanumDrive drive;
    TrajectorySequence deposit;
    TrajectorySequence L;
    TrajectorySequence C;
    TrajectorySequence R;
    TeamShippingElementDetector tagDetector;
    TeamShippingElementDetector.POSITIONS tagPosition = null;

    public double STARTX = 36;
    public double STARTY = 63;
    public double STARTTHETA = 90;
    public double DEPOSITY = 48;
    public double DEPOSITTHETA = STARTTHETA - 90;
    public double MIDX = 36;
    public double MIDY = 36;
    public double DRIFTX = -24;

    private int tagDetectionFails = 0;

    double startHeading;

    @Override
    public void init() {
        robot.init(hardwareMap, false);

        if (SetDrivingStyle.startOnRight) {DEPOSITTHETA += 180;}

        startHeading = robot.drivetrain.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, Math.toRadians(STARTTHETA));

        tagDetector = new TeamShippingElementDetector(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Pose2d depositPose = new Pose2d(MIDX, DEPOSITY, Math.toRadians(DEPOSITTHETA));

        deposit = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(MIDX, DEPOSITY, Math.toRadians(DEPOSITTHETA)), Math.toRadians(STARTTHETA+180))
                .build();

        L = drive.trajectorySequenceBuilder(depositPose)
                .lineToLinearHeading(new Pose2d(MIDX, MIDY, Math.toRadians(STARTTHETA)))
                .lineTo(new Vector2d(MIDX-DRIFTX, MIDY))
                .build();

        C = drive.trajectorySequenceBuilder(depositPose)
                .lineToLinearHeading(new Pose2d(MIDX, MIDY, Math.toRadians(STARTTHETA)))
                .build();

        R = drive.trajectorySequenceBuilder(depositPose)
                .lineToLinearHeading(new Pose2d(MIDX, MIDY, Math.toRadians(STARTTHETA)))
                .lineTo(new Vector2d(MIDX+DRIFTX, MIDY))
                .build();

        robot.deposit.intake();
        try {
            Thread.sleep(200); // TODO this is bad
        } catch (InterruptedException e) {
            // TODO twice this is very bad
        }
        robot.deposit.hold();
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
        FollowRRTraj forward = new FollowRRTraj(robot.drivetrain, drive, deposit);
        FollowRRTraj park;
        if (tagPosition == TeamShippingElementDetector.POSITIONS.ONE) {
            park = new FollowRRTraj(robot.drivetrain, drive, L);
        } else if (tagPosition == TeamShippingElementDetector.POSITIONS.THREE) {
            park = new FollowRRTraj(robot.drivetrain, drive, R);
        } else {
            park = new FollowRRTraj(robot.drivetrain, drive, C);
        }
        scheduler.schedule(false, new SequentialCommandGroup(forward,
                new DepositConeAuto(robot.slides, robot.arm, robot.deposit, 0.2, timer),
                park
        ));
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
    public void end() {
        AutoToTeleopContainer.getInstance().setAngleDelta(startHeading-robot.drivetrain.getHeading()+Math.toRadians(180));
    }
}