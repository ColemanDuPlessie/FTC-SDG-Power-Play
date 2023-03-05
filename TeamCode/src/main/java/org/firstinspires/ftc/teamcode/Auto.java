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
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.backend.CommandbasedOpmode;
import org.firstinspires.ftc.teamcode.backend.commands.DepositConeAuto;
import org.firstinspires.ftc.teamcode.backend.commands.FollowRRTraj;
import org.firstinspires.ftc.teamcode.backend.commands.IntakeConeAuto;
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
    TrajectorySequence intake;
    TrajectorySequence subsequentDeposit;
    TrajectorySequence L;
//    TrajectorySequence C;
    TrajectorySequence R;
    TrajectorySequence prepPark;
    TeamShippingElementDetector tagDetector;
    TeamShippingElementDetector.POSITIONS tagPosition = null;

    public double STARTX = 36;
    public double STARTY = 63;
    public double STARTTHETA = 90;
    public double DEPOSITX = 34;
    public double DEPOSITY = 10;
    public double DEPOSITTHETA = STARTTHETA - 45;
    public double INTAKEX = 55;
    public double INTAKEY = 12;
    public double INTAKETHETA = STARTTHETA + 90;
    public double MIDX = 36;
    public double MIDY = 12;
    public double DRIFTX = -24;
    public double INTAKEFUDGEFACTOR = -1;

    private int tagDetectionFails = 0;

    private Command depositConeCommand;
    private Command subsequentDepositConeCommand;
    private Command prepParkCommand;
    private Command intakeConeCommand;

    double startHeading;

    @Override
    public void init() {
        robot.init(hardwareMap, false);

        if (SetDrivingStyle.startOnRight) {
            DEPOSITTHETA += 90;
            DEPOSITX += 2*(MIDX-DEPOSITX);
            INTAKETHETA += 180;
            INTAKEX += 2*(MIDX-INTAKEX);
            INTAKEFUDGEFACTOR *= -1;
        }

        startHeading = robot.drivetrain.getHeading();

        Pose2d startPose = new Pose2d(STARTX, STARTY, Math.toRadians(STARTTHETA));

        tagDetector = new TeamShippingElementDetector(hardwareMap);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        Pose2d intakePose = new Pose2d(INTAKEX, INTAKEY, Math.toRadians(INTAKETHETA));

        Pose2d depositPose = new Pose2d(DEPOSITX, DEPOSITY, Math.toRadians(DEPOSITTHETA));

        Pose2d prepParkPose = new Pose2d(MIDX, MIDY, Math.toRadians(STARTTHETA));

        prepPark = drive.trajectorySequenceBuilder(depositPose)
                .lineToLinearHeading(prepParkPose)
                .build();

        prepParkCommand = new FollowRRTraj(robot.drivetrain, drive, prepPark);

        intakeConeCommand = new IntakeConeAuto(robot.arm, robot.deposit, 5, timer);
        depositConeCommand = new DepositConeAuto(robot.slides, robot.arm, robot.deposit, 0.95, timer);
        subsequentDepositConeCommand = new DepositConeAuto(robot.slides, robot.arm, robot.deposit, 0.95, timer);

        intake = drive.trajectorySequenceBuilder(prepParkPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d((INTAKEX+MIDX*2)/3, INTAKEY+2, Math.toRadians(INTAKETHETA)), Math.toRadians(INTAKETHETA+180))
                .lineToLinearHeading(intakePose)
                .build();

        deposit = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(MIDX, MIDY+12, Math.toRadians(STARTTHETA)), Math.toRadians(STARTTHETA+180))
                .addDisplacementMarker(() -> scheduler.schedule(depositConeCommand))
                .splineToSplineHeading(depositPose.plus(new Pose2d(0, -3, 0)), Math.toRadians(DEPOSITTHETA+180))
                .lineToLinearHeading(depositPose)
                .build();

        subsequentDeposit = drive.trajectorySequenceBuilder(intakePose.plus(new Pose2d(INTAKEFUDGEFACTOR, 0, 0)))
                .addDisplacementMarker(() -> drive.setPoseEstimate(drive.getPoseEstimate().plus(new Pose2d(INTAKEFUDGEFACTOR, 0, 0))))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d((MIDX+2*INTAKEX)/3, MIDY, Math.toRadians(STARTTHETA)), Math.toRadians(INTAKETHETA))
                .addDisplacementMarker(() -> scheduler.schedule(subsequentDepositConeCommand))
                .splineToSplineHeading(depositPose, Math.toRadians(DEPOSITTHETA+180))
                .build();

        L = drive.trajectorySequenceBuilder(prepParkPose)
                .lineTo(new Vector2d(MIDX-DRIFTX, MIDY))
                .build();

//        C = drive.trajectorySequenceBuilder(prepParkPose)
//                .lineTo(new Vector2d(MIDX, MIDY))
//                .build();

        R = drive.trajectorySequenceBuilder(prepParkPose)
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
        Command park;
        if (tagPosition == TeamShippingElementDetector.POSITIONS.ONE) {
            park = new FollowRRTraj(robot.drivetrain, drive, L);
        } else if (tagPosition == TeamShippingElementDetector.POSITIONS.THREE) {
            park = new FollowRRTraj(robot.drivetrain, drive, R);
        } else {
            park = new InstantCommand(() -> {});
//            park = new FollowRRTraj(robot.drivetrain, drive, C);
        }
        scheduler.schedule(false, new SequentialCommandGroup(forward,
                new WaitUntilCommand(() -> !scheduler.isScheduled(depositConeCommand)),
                new FollowRRTraj(robot.drivetrain, drive, intake),
                new ScheduleCommand(intakeConeCommand),
                new WaitUntilCommand(() -> !scheduler.isScheduled(intakeConeCommand)),
                new FollowRRTraj(robot.drivetrain, drive, subsequentDeposit),
                new WaitUntilCommand(() -> !scheduler.isScheduled(subsequentDepositConeCommand)),
                prepParkCommand,
                park,
                new InstantCommand(() -> robot.arm.setTargetPosition(0.0))
        ));
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    boolean s = false;
    boolean a = false;
    boolean d = false;

    @Override
    public void loop() {
        s = scheduler.requiring(robot.slides) != null;
        a = scheduler.requiring(robot.arm) != null;
        d = scheduler.requiring(robot.deposit) != null;
        telemetry.addData("Deposit", d);
        telemetry.addData("Slides", s);
        telemetry.addData("Arm", a);
        telemetry.addData("Drivetrain", scheduler.requiring(robot.drivetrain));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void end() {
        AutoToTeleopContainer.getInstance().setAngleDelta(startHeading-robot.drivetrain.getHeading()+Math.toRadians(180));
    }
}