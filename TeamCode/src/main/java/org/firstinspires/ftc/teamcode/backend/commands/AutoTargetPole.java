package org.firstinspires.ftc.teamcode.backend.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
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
import java.util.Objects;
import java.util.Optional;

public class AutoTargetPole extends CommandBase {

    private final DrivetrainSubsystem dt;
    private final GamepadWrapper gamepad;
    private final PoleLocalizationPipeline localizer;
    private final OpenCvCamera source;
    private final GamepadButton held;

    public static double speedMult = 0.3;
    public static double kPf = 0.005;
    public static double kIf = 0.000;
    public static double kDf = 0.015;
    public static double kPs = 0.01;
    public static double kIs = 0.0001;
    public static double kDs = 0.03;

    private final PIDController forwardPID;
    private final PIDController strafePID;

    public int successfulDetections = 0; // TODO

    private PoleDetection targetPole = new PoleDetection(Integer.MAX_VALUE, Integer.MAX_VALUE);

    private final PoleDetection target = new PoleDetection(0, 125);

    /**
     * IMPORTANT: localizer should already be initialized and connected to the camera.
     */
    public AutoTargetPole(DrivetrainSubsystem dt, CameraSubsystem camera, ElapsedTime aTimer, GamepadWrapper gamepad, GamepadButton held) {
        this.gamepad = gamepad;
        this.dt = dt;
        addRequirements(dt, camera);
        this.localizer = (PoleLocalizationPipeline) camera.pipeline;
        this.source = camera.camera;
        this.held = held;
        forwardPID = new PIDController(kPf, kIf, kDf, aTimer);
        strafePID = new PIDController(kPs, kIs, kDs, aTimer);
    }

//    @Override
//    public void initialize() {source.startStreaming(320, 240);}

    @Override
    public void execute() {
        double turn = gamepad.getRightStickX()*0.25;
        ArrayList<PoleDetection> poles = localizer.getDetections();
        Optional<PoleDetection> rawDetection = poles.stream().filter(Objects::nonNull).min(Comparator.comparing(p -> p.minus(target).getR()));
        if (rawDetection.isPresent()) {
            targetPole = rawDetection.get().minus(target);
            successfulDetections++;
        } else {
            dt.driveSimple(0, turn, 0, speedMult);
            return;
        }
        double forward = forwardPID.update(targetPole.y, 0);
        double strafe = strafePID.update(targetPole.x, 0);
        dt.driveSimple(forward, turn, strafe, speedMult);
    }

    @Override
    public boolean isFinished() { return !held.get(); }

//    @Override
//    public void end(boolean isInterrupted) {source.stopStreaming();}

    public int getError() {return targetPole.getR();}

    public void debug(Telemetry t) {
        t.addData("X dist", targetPole.x);
        t.addData("Y dist", targetPole.y);
        t.addData("Error", targetPole.getR());
        t.addData("Detections", successfulDetections);
        localizer.debug(t);
    }
}
