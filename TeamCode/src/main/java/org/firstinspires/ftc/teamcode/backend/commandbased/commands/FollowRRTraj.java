package org.firstinspires.ftc.teamcode.backend.commandbased.commands;

import org.firstinspires.ftc.teamcode.backend.commandbased.Command;
import org.firstinspires.ftc.teamcode.backend.commandbased.Subsystem;
import org.firstinspires.ftc.teamcode.backend.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.backend.roadrunner.trajectorysequence.TrajectorySequence;

public class FollowRRTraj implements Command {

    private SampleMecanumDrive drive;
    private TrajectorySequence traj;

    public FollowRRTraj(SampleMecanumDrive drive, TrajectorySequence traj) {
        this.drive = drive;
        this.traj = traj;
    }

    @Override
    public boolean isInterruptable() {
        return false;
    }

    @Override
    public Subsystem[] subsystemsUsed() {
        return new Subsystem[]{Subsystem.DRIVETRAIN};
    }

    @Override
    public void init() {
        drive.followTrajectorySequenceAsync(traj);
    }

    @Override
    public void update() {
        drive.update();
    }

    @Override
    public boolean isOver() {
        return !drive.isBusy();
    }

    @Override
    public void stop(boolean isInterrupted) {

    }
}
