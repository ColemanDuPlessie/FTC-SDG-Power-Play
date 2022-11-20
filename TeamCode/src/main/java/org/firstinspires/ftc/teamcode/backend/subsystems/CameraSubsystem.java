package org.firstinspires.ftc.teamcode.backend.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.backend.cv.pipelines.PoleLocalizationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import kotlin.NotImplementedError;

@Config
public class CameraSubsystem extends SubsystemBase {

    public OpenCvCamera camera;
    public OpenCvPipeline pipeline;

    public enum pipelineType {
        SLEEVE_DETECTOR,
        POLE_LOCALIZER
    }

    public void init(HardwareMap ahwMap, pipelineType p) {
        switch (p) {
            case SLEEVE_DETECTOR: throw new NotImplementedError();
            case POLE_LOCALIZER: pipeline = new PoleLocalizationPipeline(); break;
        }
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(ahwMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        camera.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    @Override
    public void periodic() {
    }

}
