package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.backend.cv.PoleDetection;
import org.firstinspires.ftc.teamcode.backend.cv.pipelines.PoleLocalizationPipeline;
import org.firstinspires.ftc.teamcode.backend.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.backend.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.backend.utilities.GamepadWrapper;
import org.firstinspires.ftc.teamcode.backend.utilities.controllers.PIDController;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.Comparator;

public class AutoTargetPole extends CommandBase {

    private final DrivetrainSubsystem dt;
    private final GamepadWrapper gamepad;
    private final PoleLocalizationPipeline localizer;
    private final OpenCvCamera source;

    public static double speedMult = 0.5;
    public static double kP = 0.01;
    public static double kI = 0.001;
    public static double kD = 0.004;
    public static int acceptableError = 20;

    private final PIDController forwardPID;
    private final PIDController strafePID;

    private PoleDetection targetPole;

    private final PoleDetection target = new PoleDetection(0, 100);

    /**
     * IMPORTANT: localizer should already be initialized and connected to the camera.
     */
    public AutoTargetPole(DrivetrainSubsystem dt, CameraSubsystem camera, ElapsedTime aTimer, GamepadWrapper gamepad) {
        this.gamepad = gamepad;
        this.dt = dt;
        addRequirements(dt, camera);
        this.localizer = (PoleLocalizationPipeline) camera.pipeline;
        this.source = camera.camera;
        forwardPID = new PIDController(kP, kI, kD, aTimer);
        strafePID = new PIDController(kP, kI, kD, aTimer);
    }

    @Override
    public void initialize() {source.startStreaming(320, 180);}

    @Override
    public void execute() {
        ArrayList<PoleDetection> poles = localizer.getDetections();
        targetPole = poles.stream().max(Comparator.comparing(p -> p.minus(target).getR())).get().minus(target);
        double forward = forwardPID.update(targetPole.y, 0);
        double turn = 0.0; // TODO
        double strafe = strafePID.update(targetPole.x, 0);
        dt.driveSimple(-forward, turn, -strafe, speedMult);
    }

    @Override
    public boolean isFinished() { return targetPole.getR() <= acceptableError; }

    @Override
    public void end(boolean isInterrupted) {source.stopStreaming();}

    public int getError() {return targetPole.getR();}

    public void debug(Telemetry t) {
        t.addData("X dist", targetPole.x);
        t.addData("Y dist", targetPole.y);
    }
}
